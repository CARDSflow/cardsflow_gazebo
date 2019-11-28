#include <include/cardsflow_gazebo/muscle/IMuscle.hpp>
#include "cardsflow_gazebo/muscle/IMuscle.hpp"


namespace cardsflow_gazebo {

    IMuscle::IMuscle(physics::ModelPtr parent_model) : muscleLength(0), tendonLength(0), initialTendonLength(0),
                                                       firstUpdate(true), parent_model(parent_model) {
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "MuscPlugin",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }
        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        markerMsg.set_ns("default");
        markerMsg.set_id(120);
        markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);

        ignition::msgs::Material *matMsg = markerMsg.mutable_material();
        matMsg->mutable_script()->set_name("Gazebo/Blue");
        ignition::msgs::Set(markerMsg.mutable_pose(),
                            ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
        markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
        markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);

        PID.reset(new MuscPID());

         double Kp, Ki, Kd;
         if (nh->hasParam("musclemodel_Kp")) {
             nh->getParam("musclemodel_Kp", Kp);
             PID->params[POSITION].Kp = Kp;
             PID->params[VELOCITY].Kp = Kp;
             PID->params[DISPLACEMENT].Kp = Kp;
             PID->params[FORCE].Kp = Kp;
             ROS_INFO_ONCE_NAMED("IMuscle", "using Kp %lf", Kp);
         }


         if (nh->hasParam("musclemodel_Ki")) {
             nh->getParam("musclemodel_Ki", Ki);
             PID->params[POSITION].Ki = Ki;
             PID->params[VELOCITY].Ki = Ki;
             PID->params[DISPLACEMENT].Ki = Ki;
             PID->params[FORCE].Ki = Ki;
             ROS_INFO_ONCE_NAMED("IMuscle", "using Ki %lf", Ki);
         }


         if (nh->hasParam("musclemodel_Kd")) {
             nh->getParam("musclemodel_Kd", Kd);
             PID->params[POSITION].Kd = Kd;
             PID->params[VELOCITY].Kd = Kd;
             PID->params[DISPLACEMENT].Kd = Kd;
             PID->params[FORCE].Kd = Kd;
             ROS_INFO_ONCE_NAMED("IMuscle", "using Kd %lf", Kd);
         }

    }

#ifdef ENABLE_LOGGING
    bool IMuscle::saveDataService(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res) {
        lock_guard<mutex> lock(mux);
        auto ll = log;
        nlohmann::json j(ll);
        std::ofstream o(this->name + ".json");
        o << std::setw(4) << j << std::endl;
        ROS_INFO_STREAM("Save to " << name << ".json");
        o.close();
        log.clear();
        return true;
    }
#endif

    void IMuscle::Init(MuscInfo &muscInfo) {

        /// Build Linked Viapoint list with corresponding wraping
        initViaPoints(muscInfo);

        see.see = muscInfo.see;
        name = muscInfo.name;
        string num = name.substr(5, name.back());
        muscleID = std::stoi(num.c_str());

        see.see.expansion = 0.0;
        see.see.force = 0.0;

        //TODO read motor params from SDF
        motor.setAnchorResistance(0.797);
        motor.setAnchorInductance(1.8e-4);
        motor.setBackEmfConstant(1.42e-2);
        motor.setTorqueConstant(14.2e-3);
        motor.setGearboxEfficiency(0.59);
        motor.setGearboxRatio(53);
        motor.setMomentOfInertiaGearbox(0.4e-7);
        motor.setMomentOfInertiaMotor(4.09e-7);
        motor.setSpindleRadius(0.0045);

        motor.setVoltage(0.0);
        motor.setLoadTorque(0.0);

        motor.setIntegrator(
                Actuator::RungeKutta4);
        motor.setTimeStep(0.0001);

        #ifdef ENABLE_LOGGING
        save_srv = nh->advertiseService("/roboy/simulation/SaveData/"+name, &IMuscle::saveDataService, this);
        #endif

        ROS_INFO_STREAM("Initialized muscle with id: " << muscleID);
    }

    void IMuscle::Update(ros::Time &time, ros::Duration &period) {

        if (firstUpdate) {
            period = ros::Duration(0.001);
        }

      double voltage = 0;
        if (pid_control) {
            switch (PID->control_mode) {
                case POSITION:
//                    ROS_INFO_STREAM_THROTTLE(1, "PID cmd: " << cmd << " feedback: " << feedback.position);
                    voltage = PID->calculate(period.toSec(), cmd, feedback.position);
                    break;
                case VELOCITY:
                    // ROS_INFO_STREAM_THROTTLE(1, "PID cmd: " << cmd << " feedback: " << feedback.velocity);
                    voltage = PID->calculate(period.toSec(), cmd, feedback.velocity);
                    break;
                case DISPLACEMENT:
                    actuatorForce = muscleForce = springConsts[0] + springConsts[1]*cmd;//(cmd / (0.1 * 0.001));
                    see.deltaX = springMeterPerEncoderTicks(cmd);

                    break;
                case FORCE:
                    if(cmd>=0) {
                        muscleForce = cmd;
                        // ROS_INFO_STREAM_THROTTLE(5, name << " force(N): " << cmd);
                    }else
                        muscleForce = 0;
                    break;
                default:
//                    actuator.motor.voltage = 0;
                    voltage = 0;
                    break;
            }

        } else {
            voltage = cmd*24;
        }

        markerMsg.set_id(muscleID);
        markerMsg.clear_point();

        // if (see.deltaX > 0 || (!firstUpdate && prevMuscleLength > muscleLength)) {
        //     markerMsg.mutable_material()->mutable_script()->set_name("Gazebo/Green");
        // } else {
        //     markerMsg.mutable_material()->mutable_script()->set_name("Gazebo/Blue");
        // }

        if (muscleForce > 0) {
            markerMsg.mutable_material()->mutable_script()->set_name("Gazebo/Green");
        } else {
            markerMsg.mutable_material()->mutable_script()->set_name("Gazebo/Blue");
        }

        for (int i = 0; i < viaPoints.size(); i++) {
            // update viaPoint coordinates
            // absolute position + relative position=actual position of each via point
            viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
                                              viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);

            ignition::msgs::Set(markerMsg.add_point(), viaPoints[i]->globalCoordinates);
        }


        node.Request("/marker", markerMsg);

        for (int i = 0; i < viaPoints.size(); i++) {
            viaPoints[i]->UpdateForcePoints();
        }

        muscleLength = 0.0;

        for (int i = 0; i < viaPoints.size(); i++) {
          muscleLength += viaPoints[i]->previousSegmentLength;
        }

        if(firstUpdate)
        {
            prevMuscleLength = muscleLength;
            tendonLength = initialTendonLength = muscleLength;
            firstUpdate = false;
        }


        double springDisplacement = 0;

        if (!dummy) {
            springDisplacement = muscleLength - tendonLength ;
//            if (springDisplacement > 0.08)
//                springDisplacement = 0.08;
            if (springDisplacement < 0)
                springDisplacement = 0;
            ROS_INFO_STREAM_THROTTLE(1, "Spring displacement (m) for " << name << ": "  << springDisplacement);

//            see.deltaX = springEncoderTicksPerMeter(springDisplacement);
            if (springDisplacement > 0) {
            //TODO muscleForce != actuatorForce
            muscleForce = actuatorForce = springConsts[0] + springConsts[1]*(springEncoderTicksPerMeter(springDisplacement)); //TODO move this to SEE class
            } else {
                muscleForce = actuatorForce = 0;
            }

            calculateTendonForceProgression();
            motor.setLoadTorque(motor.getSpindleRadius()*viaPoints[0]->fb);
            motor.setVoltage(voltage);
            motor.step(period.toSec());


            tendonLength = initialTendonLength - degreesToRadians((motor.getInitialPosition() - motor.getPosition()) * motor.getSpindleRadius());

            feedback.position = myoMuscleEncoderTicksPerMeter(motor.getPosition()*motor.getSpindleRadius()* 2.0 * M_PI/360.0);// winch_radius*2pi*radians = m
            feedback.velocity = myoMuscleEncoderTicksPerMeter(motor.getLinearVelocity()); // m/s
            feedback.displacement = springEncoderTicksPerMeter(springDisplacement); // m

        }
        else {
//            see.deltaX = cmd;
            calculateTendonForceProgression();
        }

//        ROS_INFO_STREAM_THROTTLE(5, "voltage: " << motor.getVoltage()
//                                                << "\t position: " << motor.getPosition()
//                                                << "\t displacement: " << springDisplacement / (0.01 * 0.001)
//                                                << "\t force: " << viaPoints[0]->fb);


    #ifdef ENABLE_LOGGING
        // log setpoint and current value
        status["cmd"] = cmd;
        status["position"] = feedback.position;
        status["velocity"] = feedback.velocity;
        status["displacement"] = feedback.displacement;
        status["force"] = actuatorForce;
        status["voltage"] =  motor.getVoltage();
        status["muscleLengthChange"] = prevMuscleLength - muscleLength;
        status["tendonLengthChangeTotal"] = initialTendonLength - tendonLength;
        status["current"] = motor.getCurrent();

        log[time.now().toNSec()] = status;
    #endif

        prevMuscleLength = muscleLength;
    }

    ////////////////////////////////////////////////////
    // The Viapoint Wraping-type gets instantiated
    // and built into a Linked list
    void IMuscle::initViaPoints(MuscInfo &muscInfo) {
        muscleName = muscInfo.name;
        // first viapoint link is parent link
        parent_link = muscInfo.viaPoints[0].link;
        for (int i = 0; i < muscInfo.viaPoints.size(); i++) {
            ViaPointInfo vp = muscInfo.viaPoints[i];
            if (vp.type == IViaPoints::FIXPOINT) {
                IViaPointsPtr ptr(new IViaPoints(vp.local_coordinates, vp.link));
                viaPoints.push_back(ptr);
            } else if (vp.type == IViaPoints::SPHERICAL) {
                SphericalWrappingPtr ptr(
                        new SphericalWrapping(vp.local_coordinates, vp.radius, vp.state, vp.revCounter, vp.link));
                viaPoints.push_back(ptr);
                ROS_INFO("state %d", vp.state);
            } else if (vp.type == IViaPoints::CYLINDRICAL) {
                ROS_INFO("state %d", vp.state);
                CylindricalWrappingPtr ptr(
                        new CylindricalWrapping(vp.local_coordinates, vp.radius, vp.state, vp.revCounter, vp.link));
                viaPoints.push_back(ptr);
                ROS_INFO("state %d", vp.state);
            } else if (vp.type == IViaPoints::MESH) {
                //TODO
            }
        }

        //linked list
        physics::Joint_V joints = parent_model->GetJoints();
        for (int i = 1; i < viaPoints.size(); i++) {
            viaPoints[i]->prevPoint = viaPoints[i - 1];
            viaPoints[i - 1]->nextPoint = viaPoints[i];
            for(physics::JointPtr joint:joints){
                physics::LinkPtr parent_link = joint->GetParent();
                physics::LinkPtr child_link = joint->GetChild();
                // if this viapoint and the previous viapoint are spanning a joint
                if((viaPoints[i]->link == child_link && viaPoints[i-1]->link == parent_link ) ||
                   (viaPoints[i]->link == parent_link && viaPoints[i-1]->link == child_link )){
                    viaPoints[i-1]->joint = joint;
                }
            }
        }
    }

    /////////////////////////////////////////////////
    // Calculates how the force goes along the tendon by going throught the Viapoint
    void IMuscle::calculateTendonForceProgression() {
        for (int i = 0; i < viaPoints.size(); i++) {

            if (viaPoints[i]->prevPoint && viaPoints[i]->nextPoint) {
                viaPoints[i]->fa = viaPoints[i]->prevPoint->fb;
                viaPoints[i]->fb = viaPoints[i]->prevPoint->fb;
            } else if (!viaPoints[i]->prevPoint) {
                viaPoints[i]->fa = 0;
                viaPoints[i]->fb = muscleForce;
            } else if (!viaPoints[i]->nextPoint) {
                viaPoints[i]->fa = viaPoints[i]->prevPoint->fb;
                viaPoints[i]->fb = 0;
            }
            //CalculateForce differs for each wraping-type
            viaPoints[i]->CalculateForce();
        }
    }

    double IMuscle::getMuscleLength()
    {
        return muscleLength;
    }

    double IMuscle::getInitialTendonLength() {
        return initialTendonLength;
    }

    double IMuscle::getTendonLength() {
        return tendonLength;
    }

    double IMuscle::getMuscleForce() {
        return muscleForce;
    }

    double IMuscle::getTendonVelocity() {
        return radiansToDegrees(motor.getAngularVelocity()*motor.getSpindleRadius());
    }
}
