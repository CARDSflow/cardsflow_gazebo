#pragma once

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include "IActuator.hpp"
#include "ISee.hpp"
#include "IViaPoints.hpp"
#include "SphericalWrapping.hpp"
#include "CylindricalWrapping.hpp"
#include "MeshWrapping.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/bind.hpp>
#include <math.h>
#include <map>
#include <vector>
#include "cardsflow_gazebo/muscle/MuscPID.hpp"
#include "cardsflow_gazebo/eccerobotMuscle/physics/Actuator.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mutex>
#include <common_utilities/CommonDefinitions.h>

namespace cardsflow_gazebo {

	using namespace std;
	using namespace boost::numeric::odeint;
	using namespace gazebo;
	using namespace Eigen;

	struct MuscInfo{
		string name;
		vector<ViaPointInfo> viaPoints;
		Motor motor;
		Gear gear;
		Spindle spindle;
		SEE see;
	};

	class IMuscle{

	public:
		IMuscle(physics::ModelPtr parent_model);

        ////////////////////
		/// \brief The Init function.
		///
		/// This function initializes the plugin.
		/// \param[in] muscInfo contains info about via points, motor, gear, spindle and see of the muscles
		void Init(MuscInfo &muscInfo);
		void Update(ros::Time &time, ros::Duration &period );
        double getMuscleLength();
        double getInitialTendonLength();
        double getTendonLength();
        double getMuscleForce();
        double getTendonVelocity(); // in m/s
		string name;
		vector<IViaPointsPtr> viaPoints;
		double cmd = 0;
		bool pid_control = true;
        bool dummy = false;
        boost::shared_ptr<MuscPID> PID;

        struct{
            double position = 0;
            double velocity = 0;
            double displacement = 0;
        }feedback;

        vector<string> link_names;

		ignition::math::Vector3d momentArm;
        physics::ModelPtr parent_model;
        physics::LinkPtr parent_link;
        boost::shared_ptr<map<string, Matrix4d>> world_to_link_transform;
	private:
        ros::NodeHandlePtr nh;

    public:
        int muscleID;
        string muscleName;
        ofstream *file;
		ISee see;
		IActuator actuator;
		Actuator motor;

    private:
        IActuator::state_type x;
		//actuatorForce ist the force generated by the motor
		double actuatorForce = 0;
    public:
		//Motorforce is the force getting applied onto the first Viapoint
		double muscleForce = 0;
    private:
		//muscleLength describes the TendonLength from the first Viapoint to the last viapoint. (TendonLength outside the myoMotor)
        double muscleLength = 0;
		// prevMuscleLength is needded to calculate the actual angVel of the motor
		double prevMuscleLength;
		//tendonLength describes the total TendonLength from then Motor until the last viapoint. (TendonLength excluding the tendon coiled up on the motor)
        double tendonLength;
		//initial TendonLength describes the initial total tendonlength
        double initialTendonLength;
		//actual angVel
		double sim_angVel;
        bool firstUpdate;
		double sinParm = 0;
		std::vector<double> springConsts = {0, 0.1} ;

		void initViaPoints( MuscInfo &myoMuscle );
		void calculateTendonForceProgression();

		mutex mux;
	};
}

