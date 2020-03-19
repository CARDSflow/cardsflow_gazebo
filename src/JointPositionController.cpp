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

            for (auto j: joint_names) {
                joints.push_back(parent_model->GetJoint(j));
            }
            joint_target_pos.resize(joint_names.size());

            jointPID.Init(10, 0, 0, 10, -10);
            ROS_INFO("update connection");


            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&JointPositionController::OnUpdate, this));
        }

    public: void JointTargetCb(const sensor_msgs::JointState &msg) {
            for (int i=0;i<joint_names.size();i++) {
                auto it = find(msg.name.begin(), msg.name.end(), joint_names[i]);
                if (it != msg.name.end()) {
                    auto id = distance(msg.name.begin(), it);
                    joint_target_pos[i] = msg.position[id];
                }
            }


        }

        // Called by the world update start event
    public: void OnUpdate()
        {
            common::Time currTime = this->parent_model->GetWorld()->SimTime();
            common::Time stepTime = currTime - this->prevUpdateTime;
            prevUpdateTime = currTime;
            for (int i=0;i<joint_target_pos.size();i++) {
                double pos_target = joint_target_pos[i];
                double pos_curr = joints[i]->Position(0);
                double max_cmd = joint_max_effort;

                double pos_err = pos_curr - pos_target;

                // compute the effort via the PID, which you will apply on the joint
                double effort_cmd = this->jointPID.Update(pos_err, stepTime);

                // check if the effort is larger than the maximum permitted one
                effort_cmd = effort_cmd > max_cmd ? max_cmd :
                             (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);

                // apply the force on the joint
                joints[i]->SetForce(0, effort_cmd);
            }


        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        ros::NodeHandlePtr nh;
        ros::Subscriber jointTarget_sub;
        event::ConnectionPtr updateConnection;
        common::Time prevUpdateTime;

        vector<double> joint_target_pos;
        double joint_max_effort=1;
        common::PID jointPID;
        vector<string> joint_names = {"head_axis0", "head_axis1", "head_axis2"};

        // Pointer to the model
        gazebo::physics::ModelPtr parent_model;
        vector<gazebo::physics::JointPtr> joints;

        boost::shared_ptr<ros::AsyncSpinner> spinner;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JointPositionController)
}