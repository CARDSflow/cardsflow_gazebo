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
        //x[0] = motorcurrent
        //x[1] = sindleAngleVel

        PID.reset(new MuscPID());

        double Kp = 200, Ki = 0, Kd = 0;
        if (nh->hasParam("Kp")) {
            nh->getParam("Kp", Kp);
            PID->params[POSITION].Kp = Kp;
            PID->params[VELOCITY].Kp = Kp;
            PID->params[DISPLACEMENT].Kp = Kp;
            PID->params[FORCE].Kp = Kp;
            ROS_INFO_ONCE_NAMED("IMuscle", "using Kp %lf", Kp);
        }
        if (nh->hasParam("Ki")) {
            nh->getParam("Ki", Ki);
            PID->params[POSITION].Ki = Ki;
            PID->params[VELOCITY].Ki = Ki;
            PID->params[DISPLACEMENT].Ki = Ki;
            PID->params[FORCE].Ki = Ki;
            ROS_INFO_ONCE_NAMED("IMuscle", "using Ki %lf", Ki);
        }
        if (nh->hasParam("Kd")) {
            nh->getParam("Kd", Kd);
            PID->params[POSITION].Kd = Kd;
            PID->params[VELOCITY].Kd = Kd;
            PID->params[DISPLACEMENT].Kd = Kd;
            PID->params[FORCE].Kd = Kd;
            ROS_INFO_ONCE_NAMED("IMuscle", "using Kd %lf", Kd);
        }

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
    }

    void IMuscle::Update(ros::Time &time, ros::Duration &period) {
        if (pid_control) {
//            muscleForce = cmd;
//            ROS_INFO_THROTTLE(1,"%s Applying cmd: %f", name.c_str(), cmd);
            switch (PID->control_mode) {
                case POSITION:
                    actuator.motor.voltage = PID->calculate(period.toSec(), cmd, feedback.position);
                    break;
                case VELOCITY:
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


        //update force points and calculate muscle length for each Viapoint
        //muscleLength is set zero and then added up again
        if (!firstUpdate) {
            prevMuscleLength = muscleLength;
        }
        muscleLength = 0;
        for (int i = 0; i < viaPoints.size(); i++) {
            viaPoints[i]->UpdateForcePoints();
            muscleLength += viaPoints[i]->previousSegmentLength;
        }

        if(!dummy) {

            //calculate elastic force
            see.ElasticElementModel(tendonLength, muscleLength);
            see.applyTendonForce(muscleForce, actuator.elasticForce);


            // calculate the approximation of gear's efficiency
            actuator.gear.appEfficiency = actuator.EfficiencyApproximation();

            // do 1 step of integration of DiffModel() at current time
            boost::numeric::odeint::integrate(
                    [this](const IActuator::state_type &x, IActuator::state_type &dxdt, double t) {
                        // This lambda function describes the differential model for the simulations of dynamics
                        // of a DC motor, a spindle, and a gear box`
                        // x[0] - motor electric current
                        // x[1] - spindle angular velocity
                        double totalIM =
                                actuator.motor.inertiaMoment + actuator.gear.inertiaMoment; // total moment of inertia
                        dxdt[0] = 1.0 / actuator.motor.inductance * (-actuator.motor.resistance * x[0]
                                                                     - actuator.motor.BEMFConst * actuator.gear.ratio *
                                                                       x[1]
                                                                     + actuator.motor.voltage);
                        dxdt[1] = actuator.motor.torqueConst * x[0] / (actuator.gear.ratio * totalIM) -
                                  actuator.spindle.radius * actuator.elasticForce /
                                  (actuator.gear.ratio * actuator.gear.ratio * totalIM * actuator.gear.appEfficiency);
                    }, x, time.toSec(), time.toSec() + period.toSec(), period.toSec());

            actuator.motor.current = x[0];
            actuator.spindle.angVel = x[1];

            // update gearposition
            actuator.gear.position += actuator.spindle.angVel * period.toSec();
            // update tendonLength
            tendonLength = initialTendonLength - 2.0 * M_PI * actuator.spindle.radius * actuator.gear.position;

            //calculate elastic force again after actuation. without the second update the motor will be a step ahead of the simulation. the spring is the comunication of force between robot and motor.
            see.ElasticElementModel(tendonLength, muscleLength);
            see.applyTendonForce(muscleForce, actuator.elasticForce);

            calculateTendonForceProgression();

            // feedback for PID-controller
            feedback.position = actuator.gear.position;
            feedback.velocity = actuator.spindle.angVel;
            feedback.displacement = see.deltaX;
        }else{ // if dummy muscle we apply the cmd as a force directly
            calculateTendonForceProgression();
            // feedback for PID-controller
//            feedback.displacement = cmd;
        }

        if (firstUpdate){
            actuator.gear.position = 0;

            stringstream str;
            str << "\n----------------" << muscleName << "---------------" << endl;
            //linked list
            physics::Joint_V joints = parent_model->GetJoints();
            for (int i = 0; i < viaPoints.size(); i++) {
                if (viaPoints[i]->joint != nullptr) {
                    str << "viapoints " << i << " and " << i + 1 << " spanning joint " << viaPoints[i]->joint->GetName()
                        << endl;
                }
            }
            for (int i = 1; i < viaPoints.size(); i++) {
                str << "segment " << i-1 << endl;
                str << "\tglobal coordinates " << i-1 << ":\t" << viaPoints[i-1]->globalCoordinates.x << " " <<
                    viaPoints[i-1]->globalCoordinates.y << " " <<
                    viaPoints[i-1]->globalCoordinates.z << endl;
                str << "\tglobal coordinates " << i << ":\t" << viaPoints[i]->globalCoordinates.x << " " <<
                    viaPoints[i]->globalCoordinates.y << " " <<
                    viaPoints[i]->globalCoordinates.z << endl;
                str << "\tsegmentlength " << viaPoints[i]->previousSegmentLength << endl;
            }

            str << "total initial tendon length " << muscleLength;
            ROS_DEBUG_STREAM(str.str());
            firstUpdate = false;
        }
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
}