#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <string>
#include <vector>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>

#include "YouBotOODL.hpp"
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

namespace YouBot
{
	using namespace RTT;

	class ArmControllerMockup: public TaskContext
	{
		public:
		ArmControllerMockup(const string& name);
		virtual ~ArmControllerMockup();

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();
			virtual void cleanupHook();

			void unfoldManipulator();
			void foldManipulator();
			void setJointAngles(vector<double>& angles, double epsilon);
			bool getJointAngles(vector<double>& angles);

		private:
			InputPort< sensor_msgs::JointState > joint_states;

			OutputPort< motion_control_msgs::JointPositions > joint_cmd_angles;

			sensor_msgs::JointState m_joint_states;

			vector<ctrl_modes> m_modes;

			motion_control_msgs::JointPositions m_joint_cmd_angles;

			OperationCaller<void(vector<ctrl_modes>) > op_setControlModes;

	};
}
