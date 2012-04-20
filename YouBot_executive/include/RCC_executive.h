/*
 * RCC_executive.h
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
namespace YouBot
{
	using namespace RTT;
	using namespace std;
	typedef std_msgs::Float64MultiArray flat_matrix_t;
	enum state_t {FULL_CONTROL, JOINT_CONTROL, CARTESIAN_CONTROL, GRAVITY_MODE, GUARDED_MOVE, RETRACT_GRIPPER};

	static const unsigned int SIZE_JOINTS_ARRAY = 8;//5 arm + 3  virtual base
	static const unsigned int SIZE_CART_SPACE=6;//

	static const unsigned int SIZE_CART_STIFFNESS=9;
	static const unsigned int SIZE_H=16;
	static const double GRIPPER_OPENING=0.022;

	static const double UNFOLD_JOINT_POSE[]={0,0,0,0,0,0,0,0};
	static const double FOLD_JOINT_POSE[]={0,0,0,-2.8,-1.1,2.5,-1.76,-2.9};

	static const double UNFOLD_CART_POSE[]={0,0,1,0,0,0};
	static const double BASIC_JOINT_STIFFNESS[]={0,0,0,10,5,5,5,5};
	static const double BASIC_CART_STIFFNESS[]={50,50,50,50,50,50,0,0,0};
	static const double EYE4[]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

	static const double RETRACT_STIFFNESS_C[]={500,500,500,0,0,0,0,0,0};
	static const double GRIPPER_SIZE=-0.2;
	static const int X_H=3;
	static const int Y_H=7;
	static const int Z_H=11;


	class YouBot_RCC_executive: public TaskContext
		{
		public:
			YouBot_RCC_executive(const string& name);
			virtual ~YouBot_RCC_executive();

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
			void unfoldArm(); 	// operation
			void foldArm(); 	// operation
			void gravityMode();	// operation
			void retractGripper(); //operation
			void openGripper();// operation
			void closeGripper();// operation
			// configurable actions
			// Cartesian space actions
			void setHvp0(vector<double> position_c);// operation
			void setHvptip(vector<double> position_c);// operation
			void setHtipCC(vector<double> position_c);// operation
			void guardMove(vector<double> force_c);// operation
			void setCartesianStiffness(vector<double> stiffness_c);// operation
			//Joint Space actions
			void setJointsState(vector<double> position_j);	// operation
			void setJointStiffness(vector<double> stiffness_j);// operation


			//readouts
			void getJointsState(vector<double>& position_j); 	// operation
			void getTip_xyzypr(vector<double>& position_c);// operation
			void getHtip0(vector<double>& H);





			// Ports and their variables
			RTT::OutputPort<flat_matrix_t> JointSpaceSetpoint;
			RTT::OutputPort<flat_matrix_t> JointSpaceStiffness;
			RTT::OutputPort<flat_matrix_t> CartSpaceSetpoint;
			RTT::OutputPort<flat_matrix_t> CartSpaceStiffness;
			RTT::OutputPort<flat_matrix_t> HtipCC;

			RTT::InputPort<flat_matrix_t> Htip0; // Homogeneous matrix
			RTT::InputPort<flat_matrix_t> JointsStates;
			RTT::InputPort<flat_matrix_t> Wtip0;

			RTT::OutputPort<motion_control_msgs::JointPositions> gripper_cmd;

			RTT::OutputPort<std::string> events;

		protected:
			void setupComponentInterface();
			void init();

			// The following are set from operations.
//			vector<double> m_position_jnt;
//			vector<double> m_stiffness_jnt;
//			vector<double> m_position_cart;
//			vector<double> m_stiffness_cart;
			vector<double> m_force_cart;
			bool checkForce;
//			vector<double> m_HtipCC_vec;

			// Variables for ports
			flat_matrix_t m_JointSpaceSetpoint;
			flat_matrix_t m_JointSpaceStiffness;
			flat_matrix_t m_Hvp0;
			flat_matrix_t m_CartSpaceStiffness;
			flat_matrix_t m_HtipCC;


			flat_matrix_t m_Htip0;
			flat_matrix_t m_JointState;
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

}//namespace YouBot
