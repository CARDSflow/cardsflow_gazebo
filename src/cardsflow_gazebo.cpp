#include "cardsflow_gazebo/cardsflow_gazebo.hpp"

int CardsflowGazebo::roboyID_generator = 0;

CardsflowGazebo::CardsflowGazebo() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "CardsflowGazebo",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &CardsflowGazebo::MotorCommand, this);
    motorStatus_pub = nh->advertise<roboy_middleware_msgs::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    joint_state_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
    floating_base_pub = nh->advertise<geometry_msgs::Pose>("/floating_base", 1);
    tendon_state_pub = nh->advertise<roboy_simulation_msgs::Tendon>("/tendon_states", 1);
    motorConfig_srv = nh->advertiseService("/roboy/middleware/MotorConfig", &CardsflowGazebo::MotorConfigService, this);
    controlMode_srv = nh->advertiseService("/roboy/middleware/ControlMode", &CardsflowGazebo::ControlModeService, this);
    emergencyStop_srv = nh->advertiseService("/roboy/middleware/EmergencyStop", &CardsflowGazebo::EmergencyStopService,
                                             this);
    torque_srv = nh->advertiseService("/roboy/middleware/TorqueControl", &CardsflowGazebo::TorqueControlService, this);

    #ifdef PROTOBUF_opensim_5fmuscles_2eproto__INCLUDED
        muscleInfoNode = transport::NodePtr(new transport::Node());
        //TODO pass gazebo world name as node name
        muscleInfoNode->Init("default");
        muscleInfoPublisher =
                this->muscleInfoNode->Advertise<msgs::OpenSimMuscles>("~/muscles", /*50*/ 10, 60);
    #endif

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();
}

CardsflowGazebo::~CardsflowGazebo() {
    motor_status_publishing = false;
    if (motor_status_publisher != nullptr)
        motor_status_publisher->join();
}

void CardsflowGazebo::Load(gazebo::physics::ModelPtr parent_, sdf::ElementPtr sdf_) {
    ROS_INFO("Loading CardsflowGazebo plugin");
    // Save pointers to the model
    parent_model = parent_;
    sdf = sdf_;

    // Error message if the model couldn't be found
    if (!parent_model) {
        ROS_ERROR_STREAM_NAMED("loadThread", "parent model is NULL");
        return;
    }
    // Check that ROS has been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    // Get all link and joint names
    links = parent_model->GetLinks();
    for (auto link : links) {
        link_names.push_back(link->GetName());
    }

    physics::WorldPtr world = physics::get_world("default");
// Get the PresetManager object from the world
    physics::PresetManagerPtr presetManager = world->GetPresetManager();

    physics::PhysicsEnginePtr physics_engine = parent_model->GetWorld()->GetPhysicsEngine();

//    // Set the solver type to quickstep in the current profile, checking for errors
//// SetCurrentProfileParam will change the current state of the physics engine
//    if (!presetManager->SetCurrentProfileParam("solver", "world")) {
//        ROS_ERROR("Couldn't set parameter, did you pass a valid key/value pair?");
//    }
    parent_model->GetWorld()->SetGravity(ignition::math::Vector3d(0,0,-9.81));

//    // Get the Gazebo solver type
//    std::string solver_type = boost::any_cast<std::string>(physics_engine->GetParam("solver_type"));
//    ROS_INFO_STREAM("gazebo solver type " << solver_type);
//
//
//
//    // Set the number of iterations in the current profile to 2000, otherwise the joints might wobble
//    presetManager->SetCurrentProfileParam("iters", 2000);
//
//
//    // Give the global CFM (Constraint Force Mixing) a positive value (default 0) making the model a bit less stiff to avoid 			jittering
//    // Positive CFM allows links to overlap each other so the collisions aren't that hard
//    physics_engine->SetParam("cfm", 0.001);
//
//    gazebo_max_step_size = physics_engine->GetMaxStepSize();
//    ROS_WARN_STREAM("Simulation max_step_size (default: 0.001): " << physics_engine->GetMaxStepSize() << " s");
//    ROS_WARN_STREAM(
//            "Simulation real time update rate (default: 1000): " << physics_engine->GetRealTimeUpdateRate() << " s");

    // Get the Gazebo simulation period
    ros::Duration gazebo_period = ros::Duration(gazebo_max_step_size);

    // Decide the plugin control period
    control_period = ros::Duration(0.1);
    control_period = gazebo_period;

    ROS_INFO("Parsing musc muscles");
    string cardsflow_xml;
    nh->getParam("cardsflow_xml",cardsflow_xml);
    if (!parseSDFusion(cardsflow_xml, musc_info, endEffectors))
        ROS_WARN("ERROR parsing musc muscles, check your cardsflow xml file.");
    numberOfMuscles = musc_info.size();
    ROS_INFO("Found %d musc muscles in cardsflow xml file", numberOfMuscles);

    muscles.clear();
    for (uint muscle = 0; muscle < musc_info.size(); muscle++) {
        muscles.push_back(
                boost::shared_ptr<cardsflow_gazebo::IMuscle>(new cardsflow_gazebo::IMuscle(parent_model)));
        muscles.back()->Init(musc_info[muscle]);
        muscles.back()->dummy = false;
        muscles.back()->muscleID = muscle;
    }

    setPoints.resize(muscles.size(), 0.0);

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&CardsflowGazebo::Update, this));

    motor_status_publisher.reset(new boost::thread(&CardsflowGazebo::MotorStatusPublisher, this));
    motor_status_publisher->detach();

    for (auto link:parent_model->GetLinks()) {
        links.push_back(link);
        link_names.push_back(link->GetName());
    }


    joint_state_msg.header.frame_id = "world";

    for (auto joint:parent_model->GetJoints()) {
        joints.push_back(joint);
        joint_names.push_back(joint->GetName());
        torques[joint->GetName()] = 0;
        joint_state_msg.name.push_back(joint->GetName());
        joint_state_msg.position.push_back(joint->GetAngle(0).Radian());
        joint_state_msg.velocity.push_back(joint->GetVelocity(0));
        joint_state_msg.effort.push_back(0);
    }

    world_to_link_transform.reset(new map<string, Matrix4d>);

    ROS_INFO("CardsflowGazebo ready now");
}

void CardsflowGazebo::Update() {
//    ROS_INFO("CardsflowGazebo::Update()");
    // Get the simulation time and period
    common::Time gz_time_now = parent_model->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;
    last_update_sim_time_ros = sim_time_ros;

    readSim(sim_time_ros, sim_period);
    writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros);

    #ifdef PROTOBUF_opensim_5fmuscles_2eproto__INCLUDED
        // TODO move it; rename it
        publishOpenSimInfo(&muscles, parent_model->GetWorld()->GetSimTime());
    #endif
}

void CardsflowGazebo::readSim(ros::Time time, ros::Duration period) {
    // get link transforms
    int i = 0;
    for (auto &link:links) {
        math::Pose pose = link->GetWorldPose();
        Matrix4d transform;
        transform.setIdentity();
        transform.block(0, 3, 3, 1) << pose.pos.x, pose.pos.y, pose.pos.z;
        Quaterniond q(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
        transform.block(0, 0, 3, 3) << q.matrix();
        (*world_to_link_transform.get())[link_names[i]] = transform;
        if(link_names[i]=="base"){
            Isometry3d iso(transform);
            geometry_msgs::Pose msg;
            tf::poseEigenToMsg(iso,msg);
            floating_base_pub.publish(msg);
        }
        i++;
    }

    joint_state_msg.header.seq = seq++;
    joint_state_msg.header.stamp = time;
    int j = 0;
    for (auto joint:parent_model->GetJoints()) {
        joint_state_msg.name[j] = joint->GetName();
        joint_state_msg.position[j] = joint->GetAngle(0).Radian();
        joint_state_msg.velocity[j] = joint->GetVelocity(0);
        joint_state_msg.effort[j] = 0;
        j++;
    }
    joint_state_pub.publish(joint_state_msg);

    for (uint muscle = 0; muscle < muscles.size(); muscle++) {
        for (int i = 0; i < muscles[muscle]->viaPoints.size(); i++) {
            math::Pose linkPose = muscles[muscle]->viaPoints[i]->link->GetWorldPose();
            muscles[muscle]->viaPoints[i]->linkPosition = linkPose.pos;
            muscles[muscle]->viaPoints[i]->linkRotation = linkPose.rot;
        }
        muscles[muscle]->world_to_link_transform = world_to_link_transform;
        muscles[muscle]->cmd = setPoints[muscle];
        muscles[muscle]->Update(time, period);
    }
}

void CardsflowGazebo::writeSim(ros::Time time, ros::Duration period) {
    auto msg = new roboy_simulation_msgs::Tendon();
    for (uint muscle = 0; muscle < muscles.size(); muscle++) {
        for (int i = 0; i < muscles[muscle]->viaPoints.size(); i++) {
            cardsflow_gazebo::IViaPointsPtr vp = muscles[muscle]->viaPoints[i];
            if (vp->prevForcePoint.IsFinite() && vp->nextForcePoint.IsFinite()) {
//                if (i==0) {
//                    ROS_INFO_STREAM("tendon 0 force: " << vp->prevForce);
//                }
                vp->link->AddForceAtWorldPosition(vp->prevForce, vp->prevForcePoint);
                vp->link->AddForceAtWorldPosition(vp->nextForce, vp->nextForcePoint);
            }
        }
    }
    if(updateTorques) {
        for (auto joint:joints) {
            joint->SetForce(0, torques[joint->GetName()]);
        }
        updateTorques = false;
    }

}

void CardsflowGazebo::Reset() {
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros = ros::Time();
    last_write_sim_time_ros = ros::Time();
    //reset acceleration of links and the actuator simulation.
}

void CardsflowGazebo::MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg) {
    if(msg->id!=5)
        return;
    // update pid setvalues
    lock_guard<mutex> lock(mux);
    for (uint i = 0; i < msg->motors.size(); i++) {
        if (msg->motors[i] < muscles.size()) {
            switch (muscles[msg->motors[i]]->PID->control_mode) {
                case POSITION:
                    muscles[msg->motors[i]]->cmd = myoMuscleMeterPerEncoderTick(msg->set_points[i]);
                    setPoints[msg->motors[i]] = myoMuscleMeterPerEncoderTick(msg->set_points[i]);
                    break;
                case VELOCITY:
                    muscles[msg->motors[i]]->cmd = myoMuscleMeterPerEncoderTick(msg->set_points[i]);
                    setPoints[msg->motors[i]] = myoMuscleMeterPerEncoderTick(msg->set_points[i]);// * RADIANS_PER_ENCODER_COUNT;// * 2.0 * M_PI / (2000.0f * 53.0f);
                    break;
                case DISPLACEMENT:
                    if (msg->set_points[i] >= 0) // negative displacement doesnt make sense
                        setPoints[i] = msg->set_points[i];
                    else
                        setPoints[i] = 0;
                    break;
                case FORCE:
                    if (msg->set_points[i] >= 0) // negative displacement doesnt make sense
                        setPoints[i] = msg->set_points[i];
                    else
                        setPoints[i] = 0;
                    break;
            }
        }
    }
}

void CardsflowGazebo::MotorStatusPublisher() {
    ros::Rate rate(100);
    while (motor_status_publishing) {
        roboy_middleware_msgs::MotorStatus msg;
        roboy_simulation_msgs::Tendon tendons;
        msg.power_sense = true;
        for (auto const &muscle:muscles) {
            msg.pwm_ref.push_back(muscle->cmd);
            msg.position.push_back(
                    myoMuscleEncoderTicksPerMeter(muscle->motor.getPosition() * (2 * M_PI * muscle->motor.getSpindleRadius() / 360))) ;// (2.0 * M_PI / (2000.0f * 53.0f))); // convert to motor ticks
            msg.velocity.push_back(myoMuscleEncoderTicksPerMeter(muscle->motor.getAngularVelocity()));
            msg.displacement.push_back(muscle->see.deltaX / (0.01 * 0.001)); // convert m to displacement ticks
            msg.current.push_back(muscle->motor.getVoltage()); // this is actually the pid result

            //TODO count spring displacement?
            tendons.force.push_back(muscle->getMuscleForce());
            tendons.l.push_back(muscle->getMuscleLength()); // or muscle length?
            tendons.ld.push_back(muscle->motor.getLinearVelocity());
        }
        motorStatus_pub.publish(msg);
        tendon_state_pub.publish(tendons);
        rate.sleep();
    }
}

bool CardsflowGazebo::MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                                    roboy_middleware_msgs::MotorConfigService::Response &res) {
    ROS_INFO("serving motor config service for %ld motors", req.config.motors.size());
    control_Parameters_t params;
    uint i = 0;
    for (auto motor:req.config.motors) {
        if (req.config.control_mode[i] < POSITION || req.config.control_mode[i] > DISPLACEMENT) {
            ROS_ERROR("trying to set an invalid control mode %d, available control modes: "
                      "[0]Position [1]Velocity [2]Displacement", req.config.control_mode[i]);
            continue;
        }

        int sim_muscle_number = req.config.id * NUMBER_OF_MOTORS_PER_FPGA + i;
        if (sim_muscle_number > muscles.size()) {
            ROS_ERROR("sim_muscle %d does not exist", req.config.id * NUMBER_OF_MOTORS_PER_FPGA + i);
            continue;
        }

//        params.outputPosMax = req.config.outputPosMax[i];
//        params.outputNegMax = req.config.outputNegMax[i];
        params.spPosMax = req.config.sp_pos_max[i];
        params.spNegMax = req.config.sp_neg_max[i];
        params.Kp = req.config.kp[i];
        params.Ki = req.config.ki[i];
        params.Kd = req.config.kd[i];
        params.forwardGain = req.config.forward_gain[i];
        params.deadBand = req.config.dead_band[i];
        params.IntegralPosMax = req.config.integral_pos_max[i];
        params.IntegralNegMax = req.config.integral_neg_max[i];

        muscles[sim_muscle_number]->PID->control_mode = req.config.control_mode[i];
        muscles[sim_muscle_number]->PID->params[req.config.control_mode[i]] = params;

        i++;
    }
    return true;
}

bool CardsflowGazebo::ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                                    roboy_middleware_msgs::ControlMode::Response &res) {
    if (!emergency_stop) {
        uint i = 0;
        switch (req.control_mode) {
            case POSITION:
                ROS_INFO("switch to POSITION control");
                for (auto &muscle:muscles) {
                    // cmd the current position
                    muscle->cmd = muscle->feedback.position;
                    muscle->PID->control_mode = POSITION;
                }
                break;
            case VELOCITY:
                ROS_INFO("switch to VELOCITY control");
                for (auto &muscle:muscles) {
                    // cmd the current velocity
                    muscle->cmd = muscle->feedback.velocity;
                    muscle->PID->control_mode = VELOCITY;
                }
                break;
            case DISPLACEMENT:
                ROS_INFO("switch to DISPLACEMENT control");
                for (auto &muscle:muscles) {
                    // cmd the current displacement
                    muscle->cmd = muscle->feedback.displacement;
                    muscle->PID->control_mode = DISPLACEMENT;
                }
                break;
            case FORCE:
                ROS_INFO("switch to FORCE control");
                for (auto &muscle:muscles) {
                    // cmd the current displacement
                    muscle->cmd = muscle->feedback.displacement;
                    muscle->PID->control_mode = FORCE;
                }
                break;
            default:
                return false;
        }
        return true;
    } else {
        return false;
    }
}

bool CardsflowGazebo::EmergencyStopService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if (req.data == 1) {
        ROS_INFO("emergency stop service called");
        emergency_stop = true;
    } else {
        ROS_INFO("resuming normal operation");
        emergency_stop = false;
    }
    return true;
}

bool CardsflowGazebo::TorqueControlService(roboy_middleware_msgs::TorqueControl::Request &req,
                            roboy_middleware_msgs::TorqueControl::Response &res){
    int i=0;
    lock_guard<mutex> lock(mux);
    for(auto joint_name:req.joint_names){
        gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_name);
        if(joint!=nullptr){
            if(isfinite(abs(req.torque[i])))
                torques[joint_name] = req.torque[i];
//            ROS_INFO("%s %f", joint_name.c_str(), torques[joint_name] );
        }
        i++;
    }
    updateTorques = true;
    return true;
}

#ifdef PROTOBUF_opensim_5fmuscles_2eproto__INCLUDED
void CardsflowGazebo::publishOpenSimInfo(vector<boost::shared_ptr<cardsflow_gazebo::IMuscle>> *muscles,
                                         common::Time simtime) {
    msgs::OpenSimMuscles muscles_msg = msgs::OpenSimMuscles();
    muscles_msg.set_robot_name("all_of_them");


    for (auto muscle : *muscles)
    {
        std::vector<gazebo::math::Vector3> muscle_path_buf;
        msgs::OpenSimMuscle* muscle_msg = muscles_msg.add_muscle();

        for (uint i = 0; i < muscle->viaPoints.size(); i++) {
            gazebo::math::Vector3 p;
            p.x = muscle->viaPoints[i]->prevForcePoint.x;
            p.y = muscle->viaPoints[i]->prevForcePoint.y;
            p.z = muscle->viaPoints[i]->prevForcePoint.z;
            muscle_path_buf.push_back(p);
            p.x = muscle->viaPoints[i]->nextForcePoint.x;
            p.y = muscle->viaPoints[i]->nextForcePoint.y;
            p.z = muscle->viaPoints[i]->nextForcePoint.z;
            muscle_path_buf.push_back(p);
        }

        GZ_ASSERT(muscle_path_buf.size() >= 2, "Muscles are supposed to have a start node and an end node");
        for (std::size_t i=0; i<muscle_path_buf.size(); ++i)
        {
            msgs::Vector3d* point = muscle_msg->add_pathpoint();
            msgs::Set(
                    point,
                    muscle_path_buf[i].Ign());
        }

        // For a bit better performance we could access the internal OpenSim::Muscle pointer directly.
        muscle_msg->set_length(muscle->getMuscleLength());
        muscle_msg->set_activation(0);
        muscle_msg->set_ismuscle(1);
    }

    msgs::Set(muscles_msg.mutable_time(), simtime);
    // std::cout <<  muscles_msg.DebugString() << std::endl;
    this->muscleInfoPublisher->Publish(muscles_msg);
}
#endif

bool CardsflowGazebo::parseSDFusion(const string &sdf, vector<cardsflow_gazebo::MuscInfo> &myoMuscles,
                               EndEffectorInfo &endEffectors) {
    // initialize TiXmlDocument doc with a string
    TiXmlDocument doc(sdf.c_str());
    if (!doc.LoadFile()) {
        ROS_FATAL("Can't parse via points file %s.", sdf.c_str());
        return false;
    }

    // Find joints in transmission tags
    TiXmlElement *root = doc.RootElement();

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *myoMuscle_it = NULL;
    for (myoMuscle_it = root->FirstChildElement("myoMuscle"); myoMuscle_it;
         myoMuscle_it = myoMuscle_it->NextSiblingElement("myoMuscle")) {
        cardsflow_gazebo::MuscInfo myoMuscle;
        if (myoMuscle_it->Attribute("name")) {
            myoMuscle.name = myoMuscle_it->Attribute("name");
            // myoMuscle joint acting on
            TiXmlElement *link_child_it = NULL;
            for (link_child_it = myoMuscle_it->FirstChildElement("link"); link_child_it;
                 link_child_it = link_child_it->NextSiblingElement("link")) {
                string linkname = link_child_it->Attribute("name");
                physics::LinkPtr link = parent_model->GetLink(linkname);
                if ((!linkname.empty()) && link) {
                    TiXmlElement *viaPoint_child_it = NULL;
                    for (viaPoint_child_it = link_child_it->FirstChildElement("viaPoint"); viaPoint_child_it;
                         viaPoint_child_it = viaPoint_child_it->NextSiblingElement("viaPoint")) {
                        cardsflow_gazebo::ViaPointInfo vp;
                        vp.link = link;
                        float x, y, z;
                        if (sscanf(viaPoint_child_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading [via point] (x y z)");
                            return false;
                        }
                        vp.local_coordinates = math::Vector3(x, y, z);
                        if (viaPoint_child_it->Attribute("type")) {
                            string type = viaPoint_child_it->Attribute("type");
                            if (type == "FIXPOINT") {
                                vp.type = cardsflow_gazebo::IViaPoints::FIXPOINT;
                            } else if (type == "SPHERICAL" || type == "CYLINDRICAL") {
                                if (viaPoint_child_it->QueryDoubleAttribute("radius", &vp.radius) != TIXML_SUCCESS) {
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading radius");
                                    return false;
                                }
                                if (viaPoint_child_it->QueryIntAttribute("state", &vp.state) != TIXML_SUCCESS) {
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading state");
                                    return false;
                                }
                                if (viaPoint_child_it->QueryIntAttribute("revCounter", &vp.revCounter) !=
                                    TIXML_SUCCESS) {
                                    ROS_ERROR_STREAM_NAMED("parser", "error reading revCounter");
                                    return false;
                                }
                                if (type == "SPHERICAL") {
                                    vp.type = cardsflow_gazebo::IViaPoints::SPHERICAL;
                                } else {
                                    vp.type = cardsflow_gazebo::IViaPoints::CYLINDRICAL;
                                }
                            } else if (type == "MESH") {
                                // TODO
                            } else {
                                ROS_ERROR_STREAM_NAMED("parser", "unknown type of via point: " + type);
                                return false;
                            }
                        } else {
                            ROS_ERROR_STREAM_NAMED("parser", "error reading type");
                            return false;
                        }
                        myoMuscle.viaPoints.push_back(vp);
                    }
                    if (myoMuscle.viaPoints.empty()) {
                        ROS_ERROR_STREAM_NAMED("parser", "No viaPoint element found in myoMuscle '"
                                << myoMuscle.name << "' link element.");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No link name attribute specified for myoMuscle'"
                            << myoMuscle.name << "'.");
                    continue;
                }
            }
            ROS_INFO("%ld viaPoints for myoMuscle %s", myoMuscle.viaPoints.size(), myoMuscle.name.c_str());

            //check if wrapping surfaces are enclosed by fixpoints
            for (int i = 0; i < myoMuscle.viaPoints.size(); i++) {
                if (i == 0 && myoMuscle.viaPoints[i].type != cardsflow_gazebo::IViaPoints::FIXPOINT) {
                    ROS_ERROR_STREAM_NAMED("parser", "muscle insertion has to be a fix point");
                    return false;
                }
                if (i == myoMuscle.viaPoints.size() - 1 &&
                    myoMuscle.viaPoints[i].type != cardsflow_gazebo::IViaPoints::FIXPOINT) {
                    ROS_ERROR_STREAM_NAMED("parser", "muscle fixation has to be a fix point");
                    return false;
                }
                if (myoMuscle.viaPoints[i].type != cardsflow_gazebo::IViaPoints::FIXPOINT) {
                    if (myoMuscle.viaPoints[i - 1].type != cardsflow_gazebo::IViaPoints::FIXPOINT
                        || myoMuscle.viaPoints[i + 1].type != cardsflow_gazebo::IViaPoints::FIXPOINT) {
                        ROS_ERROR_STREAM_NAMED("parser",
                                               "non-FIXPOINT via-points have to be enclosed by two FIXPOINT via-points");
                        return false;
                    }
                }
            }

            TiXmlElement *motor_child = myoMuscle_it->FirstChildElement("motor");
            if (motor_child) {
                // bemf_constant
                TiXmlElement *bemf_constant_child = motor_child->FirstChildElement("bemf_constant");
                if (bemf_constant_child) {
                    if (sscanf(bemf_constant_child->GetText(), "%lf", &myoMuscle.motor.BEMFConst) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading bemf_constant constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No bemf_constant element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // torque_constant
                TiXmlElement *torque_constant_child = motor_child->FirstChildElement("torque_constant");
                if (torque_constant_child) {
                    if (sscanf(torque_constant_child->GetText(), "%lf", &myoMuscle.motor.torqueConst) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading torque_constant constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No torque_constant element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // inductance
                TiXmlElement *inductance_child = motor_child->FirstChildElement("inductance");
                if (inductance_child) {
                    if (sscanf(inductance_child->GetText(), "%lf", &myoMuscle.motor.inductance) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading inductance constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No inductance element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // resistance
                TiXmlElement *resistance_child = motor_child->FirstChildElement("resistance");
                if (resistance_child) {
                    if (sscanf(resistance_child->GetText(), "%lf", &myoMuscle.motor.resistance) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading resistance constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No resistance element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
                // inertiaMoment
                TiXmlElement *inertiaMoment_child = motor_child->FirstChildElement("inertiaMoment");
                if (inertiaMoment_child) {
                    if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.motor.inertiaMoment) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
                            << myoMuscle.name << "' motor element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser", "No motor element found in myoMuscle '" << myoMuscle.name <<
                                                                                         "', using default parameters");
            }

            TiXmlElement *gear_child = myoMuscle_it->FirstChildElement("gear");
            if (gear_child) {
                // ratio
                TiXmlElement *ratio_child = gear_child->FirstChildElement("ratio");
                if (ratio_child) {
                    if (sscanf(ratio_child->GetText(), "%lf", &myoMuscle.gear.ratio) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading ratio constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No ratio element found in myoMuscle '"
                            << myoMuscle.name << "' gear element.");
                    return false;
                }
                // ratio
                TiXmlElement *efficiency_child = gear_child->FirstChildElement("efficiency");
                if (efficiency_child) {
                    if (sscanf(efficiency_child->GetText(), "%lf", &myoMuscle.gear.efficiency) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading efficiency constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No efficiency element found in myoMuscle '"
                            << myoMuscle.name << "' gear element.");
                    return false;
                }
                // inertiaMoment
                TiXmlElement *inertiaMoment_child = gear_child->FirstChildElement("inertiaMoment");
                if (inertiaMoment_child) {
                    if (sscanf(inertiaMoment_child->GetText(), "%lf", &myoMuscle.gear.inertiaMoment) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading inertiaMoment constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No inertiaMoment element found in myoMuscle '"
                            << myoMuscle.name << "' gear element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser", "No gear element found in myoMuscle '" << myoMuscle.name <<
                                                                                        "', using default parameters");
            }

            TiXmlElement *spindle_child = myoMuscle_it->FirstChildElement("spindle");
            if (spindle_child) {
                // radius
                TiXmlElement *radius_child = spindle_child->FirstChildElement("radius");
                if (radius_child) {
                    if (sscanf(radius_child->GetText(), "%lf", &myoMuscle.spindle.radius) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No radius element found in myoMuscle '"
                            << myoMuscle.name << "' spindle element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser",
                                       "No spindle element found in myoMuscle '" << myoMuscle.name <<
                                                                                 "', using default parameters");
            }

            TiXmlElement *SEE_child = myoMuscle_it->FirstChildElement("SEE");
            if (SEE_child) {
                // stiffness
                TiXmlElement *stiffness_child = SEE_child->FirstChildElement("stiffness");
                if (stiffness_child) {
                    if (sscanf(stiffness_child->GetText(), "%lf", &myoMuscle.see.stiffness) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading radius constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No stiffness element found in myoMuscle '"
                            << myoMuscle.name << "' SEE element.");
                    return false;
                }
                // length
                TiXmlElement *length_child = SEE_child->FirstChildElement("length");
                if (length_child) {
                    if (sscanf(length_child->GetText(), "%lf", &myoMuscle.see.length) != 1) {
                        ROS_ERROR_STREAM_NAMED("parser", "error reading length constant");
                        return false;
                    }
                } else {
                    ROS_ERROR_STREAM_NAMED("parser", "No length element found in myoMuscle '"
                            << myoMuscle.name << "' SEE element.");
                    return false;
                }
            } else {
                ROS_DEBUG_STREAM_NAMED("parser", "No SEE element found in myoMuscle '" << myoMuscle.name <<
                                                                                       "', using default parameters");
            }

        } else {
            ROS_ERROR_STREAM_NAMED("parser",
                                   "No name attribute specified for myoMuscle, please name the muscle in sdf file");
            return false;
        }
        myoMuscles.push_back(myoMuscle);
    }

    TiXmlElement *endeffector_it = nullptr;
    for (endeffector_it = root->FirstChildElement("endEffector"); endeffector_it;
         endeffector_it = endeffector_it->NextSiblingElement("endEffector")) {

        endEffectors.name.push_back(endeffector_it->Attribute("name"));
        physics::LinkPtr link = parent_model->GetLink(endeffector_it->Attribute("link"));
        if (link) {
            float x, y, z;
            if (sscanf(endeffector_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                ROS_ERROR_NAMED("parser", "error reading endeffector tip of link %s (x y z)", link->GetName().c_str());
                return false;
            }
            endEffectors.link.push_back(link);
            endEffectors.tip.push_back(Vector3d(x, y, z));
        }
    }
    TiXmlElement *marker_it = NULL;
    for (marker_it = root->FirstChildElement("marker"); marker_it;
         marker_it = marker_it->NextSiblingElement("marker")) {
        physics::LinkPtr link = parent_model->GetLink(marker_it->Attribute("link"));
        if (link) {
            float x, y, z;
            if (sscanf(marker_it->GetText(), "%f %f %f", &x, &y, &z) != 3) {
                ROS_ERROR_STREAM_NAMED("parser", "error reading marker point (x y z)");
                return false;
            }
            endEffectors.marker.push_back(make_pair(link, Vector3d(x, y, z)));
        }
    }
    return true;
}

GZ_REGISTER_MODEL_PLUGIN(CardsflowGazebo)
