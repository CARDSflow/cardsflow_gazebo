#include <rl-0.7.0/rl/math/Unit.h>
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

        PID.reset(new MuscPID());

        // velocity gains Kp = 200 , Ki = 5, Kd = 1

        double Kp = 200 , Ki = 5, Kd = 1;
        if (nh->hasParam("musclemodel_Kp")) {
            nh->getParam("musclemodel_Kp", Kp);
        }
        PID->params[POSITION].Kp = Kp;
        PID->params[VELOCITY].Kp = Kp;
        PID->params[DISPLACEMENT].Kp = Kp;
        PID->params[FORCE].Kp = Kp;
        ROS_INFO_ONCE_NAMED("IMuscle", "using Kp %lf", Kp);

        if (nh->hasParam("musclemodel_Ki")) {
            nh->getParam("musclemodel_Ki", Ki);
        }
        PID->params[POSITION].Ki = Ki;
        PID->params[VELOCITY].Ki = Ki;
        PID->params[DISPLACEMENT].Ki = Ki;
        PID->params[FORCE].Ki = Ki;
        ROS_INFO_ONCE_NAMED("IMuscle", "using Ki %lf", Ki);

        if (nh->hasParam("musclemodel_Kd")) {
            nh->getParam("musclemodel_Kd", Kd);
        }
        PID->params[POSITION].Kd = Kd;
        PID->params[VELOCITY].Kd = Kd;
        PID->params[DISPLACEMENT].Kd = Kd;
        PID->params[FORCE].Kd = Kd;
        ROS_INFO_ONCE_NAMED("IMuscle", "using Kd %lf", Kd);


    }

    void IMuscle::Init(MuscInfo &muscInfo) {

        //state initialization
        x[0] = 0.0;
        x[1] = 0.0;
        actuator.motor.voltage = 0.0;
        actuator.spindle.angVel = 0;

        /// Build Linked Viapoint list with corresponding wraping
        initViaPoints(muscInfo);

        actuator.motor = muscInfo.motor;
        actuator.gear = muscInfo.gear;
        actuator.spindle = muscInfo.spindle;
        see.see = muscInfo.see;
        name = muscInfo.name;
        see.see.expansion = 0.0;
        see.see.force = 0.0;

        motor.setAnchorResistance(0.516);
        motor.setAnchorInductance(5.73e-5);
        motor.setBackEmfConstant(11.5e-3);
        motor.setTorqueConstant(11.5e-3);
        motor.setGearboxEfficiency(0.59);
        motor.setGearboxRatio(128);
        motor.setMomentOfInertiaGearbox(0.4e-7);
        motor.setMomentOfInertiaMotor(14.5e-7);
        motor.setSpindleRadius(0.0045);

        motor.setVoltage(0.0);
        motor.setLoadTorque(0.0);

        //motor.setIntegrator((de::caliper::sim::ext::actuators::er::physics::Actuator::Integrator) atoi(argv[1]));
        motor.setIntegrator(
                Actuator::RungeKutta4);
        motor.setTimeStep(0.0001);

    }

    void IMuscle::Update(ros::Time &time, ros::Duration &period) {


      double Ki_new, Kp_new, Kd_new;
      if (nh->hasParam("musclemodel_Kp")) {
        nh->getParam("musclemodel_Kp", Kp_new);
        if (Kp_new != PID->params[POSITION].Kp)
        {
          PID->params[POSITION].Kp = Kp_new;
          PID->params[VELOCITY].Kp = Kp_new;
          PID->params[DISPLACEMENT].Kp = Kp_new;
          PID->params[FORCE].Kp = Kp_new;
          ROS_INFO_NAMED("IMuscle", "using Kp %lf", Kp_new);
        }
      }



      ROS_INFO_ONCE_NAMED("IMuscle", "using Kp %lf", Kp_new);

      if (nh->hasParam("musclemodel_Ki")) {

          nh->getParam("musclemodel_Ki", Ki_new);
          if (Ki_new != PID->params[POSITION].Ki)
          {
            PID->params[POSITION].Ki = Ki_new;
            PID->params[VELOCITY].Ki = Ki_new;
            PID->params[DISPLACEMENT].Ki = Ki_new;
            PID->params[FORCE].Ki = Ki_new;
            ROS_INFO_ONCE("IMuscle", "using Ki %lf", Ki_new);
          }
      }

      ROS_INFO_ONCE_NAMED("IMuscle", "using Ki %lf", Ki_new);

      if (nh->hasParam("musclemodel_Kd")) {
        nh->getParam("musclemodel_Kd", Kd_new);
        if (Kd_new != PID->params[POSITION].Kd)
        {
          PID->params[POSITION].Kd = Kd_new;
          PID->params[VELOCITY].Kd = Kd_new;
          PID->params[DISPLACEMENT].Kd = Kd_new;
          PID->params[FORCE].Kd = Kd_new;
          ROS_INFO("IMuscle", "using Kd %lf", Kd_new);
        }
      }

      ROS_INFO_ONCE_NAMED("IMuscle", "using Kd %lf", Kd_new);

        if (pid_control) {
//            muscleForce = cmd;
//            ROS_INFO_THROTTLE(1,"%s Applying cmd: %f", name.c_str(), cmd);
            switch (PID->control_mode) {
                case POSITION:
                    ROS_INFO_STREAM_THROTTLE(1, "PID cmd: " << cmd << " feedback: " << feedback.position);
                    actuator.motor.voltage = PID->calculate(period.toSec(), cmd, feedback.position);
                    break;
                case VELOCITY:
                    // ROS_INFO_STREAM_THROTTLE(1, "PID cmd: " << cmd << " feedback: " << feedback.velocity);
                    actuator.motor.voltage = PID->calculate(period.toSec(), cmd, feedback.velocity);
                    break;
                case DISPLACEMENT:
                    if(cmd>=0) // negative displacement doesnt make sense
                        actuator.motor.voltage = PID->calculate(period.toSec(), cmd, feedback.displacement);
                    else
                        actuator.motor.voltage = PID->calculate(period.toSec(), 0, feedback.displacement);
                    break;
                case FORCE:
//                    if(cmd>0)
                        muscleForce = cmd;
                        ROS_DEBUG_THROTTLE(1,"Applying cmd: %f", cmd);
//                    else
//                        muscleForce = 0;
                    break;
                default:
                    actuator.motor.voltage = 0;
                    break;
            }

        } else {
            actuator.motor.voltage = cmd * 24;//simulated PWM
        }


        for (int i = 0; i < viaPoints.size(); i++) {
            // update viaPoint coordinates
            // absolute position + relative position=actual position of each via point
            viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
                                              viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);


//            switch (viaPoints[i]->type){
//                case IViaPoints::FIXPOINT:
//                    viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
//                                                      viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);
//                    break;
//                case IViaPoints::CYLINDRICAL:
//                    if(this->spanningJoint!=nullptr)
//                        viaPoints[i]->globalCoordinates = this->spanningJoint->GetWorldPose().pos;
//                    break;
//                case IViaPoints::SPHERICAL:
//                    if(this->spanningJoint!=nullptr)
//                        viaPoints[i]->globalCoordinates = this->spanningJoint->GetWorldPose().pos;
//                    break;
//                case IViaPoints::MESH:
//                    viaPoints[i]->globalCoordinates = viaPoints[i]->linkPosition +
//                                                      viaPoints[i]->linkRotation.RotateVector(viaPoints[i]->localCoordinates);
//                    break;
//            }
        }

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
            see.deltaX = springDisplacement;
            if (springDisplacement > 0) {
                muscleForce = actuatorForce = springConsts[0] + springConsts[1]*(springDisplacement / (0.1 * 0.001)); //TODO move this to SEE class
            } else {
                muscleForce = actuatorForce = 0;
            }

            calculateTendonForceProgression();
            motor.setLoadTorque(motor.getSpindleRadius()*viaPoints[0]->fb);
            motor.setVoltage(actuator.motor.voltage);
            motor.step(period.toSec());

            lock_guard<mutex> lock(mux);

            tendonLength = initialTendonLength - (motor.getInitialPosition() - motor.getPosition()) *
                    rl::math::DEG2RAD * motor.getSpindleRadius();

            feedback.position = motor.getPosition();// radians
            feedback.velocity = motor.getLinearVelocity(); // m/s
            feedback.displacement = springDisplacement; // m

        }
        else {
            calculateTendonForceProgression();
            feedback.displacement = cmd;
        }

//        ROS_INFO_STREAM_THROTTLE(5, "voltage: " << motor.getVoltage()
//                                                << "\t position: " << motor.getPosition()
//                                                << "\t displacement: " << springDisplacement / (0.01 * 0.001)
//                                                << "\t force: " << viaPoints[0]->fb);


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
        return motor.getAngularVelocity()*motor.getSpindleRadius()*rl::math::RAD2DEG;
    }
}
