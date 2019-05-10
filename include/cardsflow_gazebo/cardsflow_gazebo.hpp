#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <sensor_msgs/JointState.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/TorqueControl.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <roboy_simulation_msgs/Tendon.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include "cardsflow_gazebo/muscle/IMuscle.hpp"
#include <mutex>
#include <eigen_conversions/eigen_msg.h>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include <tinyxml.h>

using namespace gazebo;
using namespace std;
using namespace chrono;
using namespace Eigen;

struct EndEffectorInfo{
    vector<string> name;
    vector<physics::LinkPtr> link;
    vector<Vector3d> tip;
    vector<pair<physics::LinkPtr,Vector3d>> marker;
};

class CardsflowGazebo : public gazebo::ModelPlugin {
public:
    /** Constructor */
    CardsflowGazebo();

    /** Destructor */
    ~CardsflowGazebo();

    /**
     * Overloaded Gazebo entry point
     * @param parent model pointer
     * @param sdf element
     */
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    /** Called at each sim step */
    void Update();

    /**
     * Read from Simulation
     * @param time current time
     * @param period period since last read
     */
    void readSim(ros::Time time, ros::Duration period);

    void readSim(gazebo::common::Time, gazebo::common::Time);

    /** Write to Simulation
     * @param time current time
     * @param period period since last read
     */
    void writeSim(ros::Time time, ros::Duration period);

    void writeSim(gazebo::common::Time, gazebo::common::Time);

    /** Called on world reset */
    void Reset();

    /** using protobuf msgs to publish tendon info to gazebo (NRP version */
    void publishOpenSimInfo(vector<boost::shared_ptr<cardsflow_gazebo::IMuscle>> *muscles, common::Time simtime);

private:
    /** This function parses a sdf string for myoMuscle parameters
     * @param sdf string
     * @param myoMuscles will be populated with the cable routing information
     * @param endEffector will be filled with endeffector information
     * @return success
     */
    bool parseSDFusion(const string &sdf, vector<cardsflow_gazebo::MuscInfo> &muscles,
                           EndEffectorInfo &endEffectors);

    /**
     *   Callback for motor commands
     *   @param msg
    */
    void MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg);

    void StepCb(const std_msgs::Int32::ConstPtr &msg);
    void DetachCb(const std_msgs::Bool::ConstPtr &msg);

    void MotorStatusPublisher();

    bool MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                            roboy_middleware_msgs::MotorConfigService::Response &res);

    bool ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                            roboy_middleware_msgs::ControlMode::Response &res);

    bool EmergencyStopService(std_srvs::SetBool::Request &req,
                              std_srvs::SetBool::Response &res);

    bool TorqueControlService(roboy_middleware_msgs::TorqueControl::Request &req,
                            roboy_middleware_msgs::TorqueControl::Response &res);

    bool DetachJointService(std_srvs::Trigger::Request &req,
                                             std_srvs::Trigger::Response &res);

    bool AtachJointService(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res);

    bool DoStep(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res);

    bool emergency_stop = false;

    static int roboyID_generator;
    ros::NodeHandle* nh;
    ros::Subscriber motorCommand_sub, pid_control_sub, step_sub, detach_sub;
    ros::Publisher motorStatus_pub, joint_state_pub, floating_base_pub, tendon_state_pub;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, torque_srv, joint_srv, step_srv, atach_srv;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::ServiceClient pauseGazebo;

    bool motor_status_publishing = true;
    boost::shared_ptr<boost::thread> motor_status_publisher = nullptr;
    boost::shared_ptr<map<string, Matrix4d>> world_to_link_transform;

    ros::Duration control_period;
    ros::Time last_update_sim_time_ros;
    ros::Time last_write_sim_time_ros;

    // Pointer to the model
    gazebo::physics::ModelPtr parent_model;
    sdf::ElementPtr sdf;
    ignition::math::Pose3<double> pose;

    double gazebo_max_step_size = 0.003;
    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection;

    uint numberOfMuscles;

    vector<double> setPoints;

    vector<boost::shared_ptr<cardsflow_gazebo::IMuscle>> muscles;
    vector<cardsflow_gazebo::MuscInfo> musc_info;
    EndEffectorInfo endEffectors;
    sensor_msgs::JointState joint_state_msg;

    map<string,double> torques;
    bool updateTorques = false;
    vector<string> link_names, joint_names;
    vector<gazebo::physics::JointPtr> joints;
    vector<gazebo::physics::LinkPtr> links;
    vector<double> joint_pos, joint_vel, joint_vel_prev, joint_acc; /// of kinematic chain
    int seq = 0;
    mutex mux;

    bool draw_gazebo_tendons = false;

    transport::NodePtr gzNode;
    transport::PublisherPtr gzPublisher;
};
