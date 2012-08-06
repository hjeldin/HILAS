#pragma once

#include <std_msgs/Float64MultiArray.h>

namespace YouBot
{
	typedef std_msgs::Float64MultiArray flat_matrix_t;
	enum state_t {FULL_CONTROL, JOINT_CONTROL, CARTESIAN_CONTROL, GRAVITY_MODE, GUARDED_MOVE, RETRACT_GRIPPER};

	static const unsigned int SIZE_ARM_JOINTS_ARRAY = 5;
	static const unsigned int SIZE_BASE_JOINTS_ARRAY = 3;
	static const unsigned int SIZE_CART_SPACE=6;//

	static const unsigned int SIZE_CART_STIFFNESS=9;
	static const unsigned int SIZE_H=16;
	static const double GRIPPER_OPENING=0.022;

	static const double UNFOLD_JOINT_POSE[]={0,0,0,0,0};
	static const double FOLD_JOINT_POSE[]={-2.8,-1.1,2.5,-1.76,-2.9};

	static const double UNFOLD_CART_POSE[]={0,0,1,0,0,0};
//	static const double BASIC_JOINT_STIFFNESS[]={0,0,0,10,5,5,5,5};
	static const double BASIC_CART_STIFFNESS[]={50,50,50,50,50,50,0,0,0};
	static const double EYE4[]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

//	static const double RETRACT_STIFFNESS_C[]={500,500,500,0,0,0,0,0,0};
	static const double GRIPPER_SIZE=-0.2;
	static const int X_H=3;
	static const int Y_H=7;
	static const int Z_H=11;

	static const size_t max_event_length = 50; // 50 is long enough for any event string exchanged
}
