/*
 * YouBot_executive.h
 *
 *  Created on: Dec 15, 2011
 *      Author: Yury Brodskiy
 */
#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <motion_control_msgs/JointPositions.h>

#include "ExecutiveTypes.hpp"

namespace YouBot
{

using namespace RTT;
using namespace std;

class YouBot_executive: public TaskContext
{
	public:
		YouBot_executive(const string& name);
		virtual ~YouBot_executive();

		virtual void updateHook();

		void stateTransition(state_t new_state);
		void stateFullControl();
		void stateGravityMode();
		void stateJointControl();
		void stateCartesianControl();
		void stateGuardedMove();

		void doneEvent();
		void checkForceEvents();

		// predefined actions
		void unfoldArm();
		void foldArm(); 	
		void gravityMode();	
//		void retractGripper(); 
		void openGripper();
		void closeGripper();

		// configurable actions
		// Cartesian space actions
		void setHvp0(vector<double> position_c);
//		void setHvptip(vector<double> position_c);
		void setHtipCC(vector<double> position_c);
		void guardMove(vector<double> force_c);
		void setCartesianStiffness(vector<double> stiffness_c);
		//Joint Space actions
		void setJointAngles(vector<double> position_j);	
		void setJointStiffness(vector<double> stiffness_j);

		//readouts
		void getJointStates(vector<double>& sample);
		void getTip_xyzypr(vector<double>& sample);
		void getHtip0(vector<double>& sample_H);

		// hacks
		void sleep(double seconds);

		// Ports and their variables
		RTT::OutputPort<flat_matrix_t> JointSpaceSetpoint;
		RTT::OutputPort<flat_matrix_t> JointSpaceStiffness;
		RTT::OutputPort<flat_matrix_t> CartSpaceSetpoint;
		RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;
		RTT::OutputPort<flat_matrix_t> HtipCC;

		RTT::InputPort<flat_matrix_t> Htip0; 
		RTT::InputPort<flat_matrix_t> JointStates;
		RTT::InputPort<flat_matrix_t> Wtip0;

		RTT::OutputPort<motion_control_msgs::JointPositions> gripper_cmd;

		RTT::OutputPort<std::string> events;

	protected:
		void setupComponentInterface();
		void init();

		// The following are set from operations.
		vector<double> m_force_cart;
		bool checkForce;

		// Variables for ports
		flat_matrix_t m_JointState;
		flat_matrix_t m_JointSpaceSetpoint;
		flat_matrix_t m_JointSpaceStiffness;

		flat_matrix_t m_Htip0;

		flat_matrix_t m_Hvp0;
		flat_matrix_t m_CartSpaceStiffness;
		flat_matrix_t m_HtipCC;			

		flat_matrix_t m_Wtip0;

		motion_control_msgs::JointPositions m_gripper_cmd;

		state_t m_state;

		std::string m_events;
	private:
		bool isForceOverLimit_Norm();
		bool isForceOverLimit_X();
		bool isForceOverLimit_Y();
		bool isForceOverLimit_Z();

};

}
