#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <sensor_msgs/JointState.h>

using namespace std;
namespace gazebo
{
    class JointPositionController : public ModelPlugin
    {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {

            physics::WorldPtr world = physics::get_world("default");
            // Get the PresetManager object from the world
            physics::PresetManagerPtr presetManager = world->PresetMgr();

            physics::PhysicsEnginePtr physics_engine = _parent->GetWorld()->Physics();

            // Set the number of iterations in the current profile to 2000, otherwise the joints might wobble
//            presetManager->SetCurrentProfileParam("iters", 2000);

            // Give the global CFM (Constraint Force Mixing) a positive value (default 0) making the model a bit less stiff to avoid 			jittering
            // Positive CFM allows links to overlap each other so the collisions aren't that hard
//            physics_engine->SetParam("cfm", 0.001);

            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "CardsflowGazebo",
                          ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }
            nh = ros::NodeHandlePtr(new ros::NodeHandle);

            jointTarget_sub = nh->subscribe("/joint_target", 1, &JointPositionController::JointTargetCb, this);


            spinner.reset(new ros::AsyncSpinner(2));
            spinner->start();

            // Store the pointer to the model
            this->parent_model = _parent;


            joints.push_back(parent_model->GetJoint("head_axis0"));
            joints.push_back(parent_model->GetJoint("head_axis1"));
            joints.push_back(parent_model->GetJoint("head_axis2"));

            jointPID.Init(10, 0, 0, 10, -10);
            ROS_INFO("update connection");


            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&JointPositionController::OnUpdate, this));
        }

    public:     void JointTargetCb(const sensor_msgs::JointState &msg) {
            auto it = find(msg.name.begin(), msg.name.end(), "head_axis0");
            if (it != msg.name.end()) {
                auto id = distance(msg.name.begin(), it);
                joint_target_pos = msg.position[id];
            }

        }

        // Called by the world update start event
    public: void OnUpdate()
        {
            common::Time currTime = this->parent_model->GetWorld()->SimTime();
            common::Time stepTime = currTime - this->prevUpdateTime;
            prevUpdateTime = currTime;


            double pos_target = joint_target_pos;
            double pos_curr = joints[0]->Position(0);
            double max_cmd = joint_max_effort;

            double pos_err = pos_curr - pos_target;

            // compute the effort via the PID, which you will apply on the joint
            double effort_cmd = this->jointPID.Update(pos_err, stepTime);

            // check if the effort is larger than the maximum permitted one
            effort_cmd = effort_cmd > max_cmd ? max_cmd :
                         (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);

            // apply the force on the joint
            joints[0]->SetForce(0, effort_cmd);
        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        ros::NodeHandlePtr nh;
        ros::Subscriber jointTarget_sub;
        event::ConnectionPtr updateConnection;
        common::Time prevUpdateTime;

        double joint_target_pos;
        double joint_max_effort=1;
        common::PID jointPID;

        // Pointer to the model
        gazebo::physics::ModelPtr parent_model;
        vector<gazebo::physics::JointPtr> joints;

        boost::shared_ptr<ros::AsyncSpinner> spinner;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JointPositionController)
}
//
//#include <ros/ros.h>
//#include <functional>
//#include <gazebo/gazebo.hh>
//#include <gazebo/physics/Model.hh>
//#include <gazebo/physics/World.hh>
//#include <gazebo/physics/Joint.hh>
//#include <sensor_msgs/JointState.h>
//
//using namespace gazebo;
//using namespace std;
//class JointPositionController : public gazebo::ModelPlugin {
//
//private:
//
//    ros::NodeHandlePtr nh;
//    ros::Subscriber jointTarget_sub;
////    ros::Publisher motorStatus_pub, joint_state_pub, floating_base_pub, tendon_state_pub;
////    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, torque_srv, attach_srv, detach_srv, mass_srv;
//    boost::shared_ptr<ros::AsyncSpinner> spinner;
//
//
//    ros::Duration control_period;
//    ros::Time last_update_sim_time_ros;
//    ros::Time last_write_sim_time_ros;
//    common::Time prevUpdateTime;
//
//    double joint_target_pos;
//    double joint_max_effort=1;
//
//    // Pointer to the model
//    gazebo::physics::ModelPtr parent_model;
//    sdf::ElementPtr sdf;
//
//    double gazebo_max_step_size = 0.003;
//    // Pointer to the update event connection
//    gazebo::event::ConnectionPtr update_connection;
//
//
//    vector<double> setPoints;
//
//    sensor_msgs::JointState joint_state_msg;
//
//
//    vector<string> link_names, joint_names;
//    vector<gazebo::physics::JointPtr> joints;
//    vector<gazebo::physics::LinkPtr> links;
//    vector<double> joint_pos, joint_vel, joint_vel_prev, joint_acc; /// of kinematic chain
//    int seq = 0;
//    mutex mux;
//
//    bool draw_gazebo_tendons = false;
//
//    /// \brief Node for protobuf communication.
//    transport::NodePtr muscleInfoNode;
//
//    /// \brief Info publisher on opensim muscles.
//    transport::PublisherPtr muscleInfoPublisher;
//
//public:
//    /** Constructor */
//    JointPositionController(){
//        ROS_INFO("Constructor");
//        if (!ros::isInitialized()) {
//            int argc = 0;
//            char **argv = NULL;
//            ros::init(argc, argv, "CardsflowGazebo",
//                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
//        }
//        nh = ros::NodeHandlePtr(new ros::NodeHandle);
//
//        jointTarget_sub = nh->subscribe("/joint_target", 1, &JointPositionController::JointTargetCb, this);
//
//
//        spinner.reset(new ros::AsyncSpinner(2));
//        spinner->start();
//    }
//
//    /** Destructor */
//    ~JointPositionController();
//
//
//
//    /**
//     * Overloaded Gazebo entry point
//     * @param parent model pointer
//     * @param sdf element
//     */
//    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf){
//        parent_model = parent;
//        joints.push_back(parent_model->GetJoint("head_axis0"));
//        joints.push_back(parent_model->GetJoint("head_axis1"));
//        joints.push_back(parent_model->GetJoint("head_axis2"));
//
//        jointPID.Init(10, 0, 0, 10, -10);
//        ROS_INFO("update connection");
//
//        update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&JointPositionController::Update, this));
//
//    }
//
//    /** Called at each sim step */
//    void Update() {
//
//
//    }
//
////    /**
////     * Read from Simulation
////     * @param time current time
////     * @param period period since last read
////     */
////    void readSim(ros::Time time, ros::Duration period);
////
////    void readSim(gazebo::common::Time, gazebo::common::Time);
////
////    /** Write to Simulation
////     * @param time current time
////     * @param period period since last read
////     */
////    void writeSim(ros::Time time, ros::Duration period);
////
////    void writeSim(gazebo::common::Time, gazebo::common::Time);
////
////    /** Called on world reset */
////    void Reset();
//
//
//
//};
//
//GZ_REGISTER_MODEL_PLUGIN(JointPositionController);
//
//
