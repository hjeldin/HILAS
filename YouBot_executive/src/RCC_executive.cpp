/*
 * RCC_executive.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: Yury Brodskiy
 */
#include "RCC_executive.h"

#include <tf/transform_broadcaster.h>
#include "ExecutiveHelpers.hpp"

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

YouBot_RCC_executive::YouBot_RCC_executive(const string& name) :
	TaskContext(name)
{
	this->init();

	this->setupComponentInterface();
}

YouBot_RCC_executive::~YouBot_RCC_executive()
{ }

void YouBot_RCC_executive::setupComponentInterface()
{
	// predefined actions
	this->addOperation("unfoldArm", &YouBot_RCC_executive::unfoldArm, this).doc("Unfold the arm. Takes no arguments");
	this->addOperation("foldArm", &YouBot_RCC_executive::foldArm, this).doc("Fold the arm. Takes no arguments");
	this->addOperation("gravityMode", &YouBot_RCC_executive::gravityMode, this).doc("Set gravity compensation mode. Takes no arguments");
	this->addOperation("openGripper", &YouBot_RCC_executive::openGripper, this).doc("Set gripper gap  to maximum. Takes no arguments");
	this->addOperation("closeGripper", &YouBot_RCC_executive::closeGripper, this).doc("Set gripper gap  to minimum. Takes no arguments");
	this->addOperation("retractGripper",&YouBot_RCC_executive::retractGripper,this).doc("Move tip for gripper length backwards. Takes no arguments");
	/*configurable actions*/
	this->addOperation("setJointsState", &YouBot_RCC_executive::setJointsState, this).doc("Joint space control");

	this->addOperation("setHvp0", &YouBot_RCC_executive::setHvp0, this).doc("Define attraction point with respect to inertial frame. Takes vector flatten H matrix");
	this->addOperation("setHvptip", &YouBot_RCC_executive::setHvptip, this).doc("Define attraction point with respect to current tool tip frame.Takes vector flatten H matrix");
	this->addOperation("setHtipCC", &YouBot_RCC_executive::setHtipCC, this).doc("Define center of stiffness and principal axes with respect to tool tip frame.Takes vector flatten H matrix");

	this->addOperation("setCartesianStiffness", &YouBot_RCC_executive::setCartesianStiffness,this).doc(" ");
	this->addOperation("setJointStiffness", &YouBot_RCC_executive::setJointStiffness,this).doc(" ");
	//measurement actions
	this->addOperation("getJointsState", &YouBot_RCC_executive::getJointsState, this).doc("Get joint space positions");
	this->addOperation("getTip_xyzypr", &YouBot_RCC_executive::getTip_xyzypr,this).doc("DEPRECATED -- Get tool tip frame in xyz roll pitch yaw. ");
	this->addOperation("getHtip0", &YouBot_RCC_executive::getHtip0,this).doc("Get tool tip frame. Returns vector flatten H matrix");


	this->addOperation("guardMove", &YouBot_RCC_executive::guardMove, this).doc(
			"Performs guarded move based on the set force, issue event e_done. NOTE: the edge of working envelope considered as obstacle. If the force 	limit is higher then max force allowed in controller e_done event will be never sent.");

	this->addPort("JointSpaceSetpoint", JointSpaceSetpoint).doc("");
	this->addPort("JointSpaceStiffness", JointSpaceStiffness).doc("");
	this->addPort("CartSpaceSetpoint", CartSpaceSetpoint).doc("");
	this->addPort("CartSpaceStiffness", CartSpaceStiffness).doc("");
	this->addPort("HtipCC", HtipCC).doc("");


	this->addPort("GripperPose", Htip0).doc("");
	this->addPort("ArmPose", JointsStates).doc("");
	this->addPort("CartForceState", Wtip0).doc("Input from the control");

	this->addPort("gripper_cmd", gripper_cmd).doc("");
	this->addPort("events_out", events).doc("");


	// Debugging/introspection properties
	this->addProperty("Joint_position_setpoint", m_JointSpaceSetpoint.data);
	this->addProperty("Joint_stiffness_setpoint", m_JointSpaceStiffness.data);
	this->addProperty("Gripper_position_setpoint", m_Hvp0.data);
	this->addProperty("Gripper_stiffness_setpoint", m_CartSpaceStiffness.data);
	this->addProperty("Force_guard",m_Wtip0.data);
	this->addProperty("HtipCC",m_HtipCC.data);
}

void YouBot_RCC_executive::init()
{
//	m_position_jnt.resize(SIZE_JOINTS_ARRAY, 0.0);
//	m_stiffness_jnt.resize(SIZE_JOINTS_ARRAY, 0.0);
//	m_position_cart.resize(SIZE_CART_SPACE, 0.0);
//	m_stiffness_cart.resize(SIZE_CART_STIFFNESS, 0.0);
//	m_force_cart.resize(SIZE_CART_SPACE, 0.0);

	// Variables for ports
	m_JointSpaceSetpoint.data.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_JointSpaceStiffness.data.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_CartSpaceStiffness.data.resize(SIZE_CART_STIFFNESS, 0.0);

	m_Hvp0.data.resize(SIZE_H, 0.0);
	m_JointState.data.resize(SIZE_JOINTS_ARRAY, 0.0);
	m_Wtip0.data.resize(SIZE_CART_SPACE, 0.0);
	m_HtipCC.data.assign(EYE4, EYE4+SIZE_H);

	m_gripper_cmd.positions.resize(1, 0.0);

	m_state = GRAVITY_MODE;
	// 50 is long enough for any event string exchanged
	m_events.reserve(50);

	JointSpaceSetpoint.setDataSample(m_JointSpaceSetpoint);
	JointSpaceStiffness.setDataSample(m_JointSpaceStiffness);
	CartSpaceSetpoint.setDataSample(m_Hvp0);
	CartSpaceStiffness.setDataSample(m_CartSpaceStiffness);
	HtipCC.setDataSample(m_HtipCC);
	gripper_cmd.setDataSample(m_gripper_cmd);
}

void YouBot_RCC_executive::openGripper()
{
	m_gripper_cmd.positions[0] = GRIPPER_OPENING;
	gripper_cmd.write(m_gripper_cmd);
}

void YouBot_RCC_executive::closeGripper()
{
	m_gripper_cmd.positions[0] = 0.0001;
	gripper_cmd.write(m_gripper_cmd);
}

void YouBot_RCC_executive::unfoldArm()
{
	RTT::log(Info) << "Call unfoldArm" << endlog();
	m_JointSpaceSetpoint.data.assign(UNFOLD_JOINT_POSE,UNFOLD_JOINT_POSE+SIZE_JOINTS_ARRAY);
	m_JointSpaceStiffness.data.assign(BASIC_JOINT_STIFFNESS,BASIC_JOINT_STIFFNESS+SIZE_JOINTS_ARRAY);
	m_HtipCC.data.assign(EYE4,EYE4+SIZE_H);
	stateTransition(FULL_CONTROL);
}
void YouBot_RCC_executive::foldArm()
{
	RTT::log(Info) << "Call foldArm" << endlog();
	m_JointSpaceSetpoint.data.assign(FOLD_JOINT_POSE,FOLD_JOINT_POSE+SIZE_JOINTS_ARRAY);
	m_JointSpaceStiffness.data.assign(BASIC_JOINT_STIFFNESS,BASIC_JOINT_STIFFNESS+SIZE_JOINTS_ARRAY);
	m_HtipCC.data.assign(EYE4,EYE4+SIZE_H);
	stateTransition(FULL_CONTROL);
}

void YouBot_RCC_executive::gravityMode()
{
	stateTransition(GRAVITY_MODE);
}

void YouBot_RCC_executive::setCartesianStiffness(vector<double> stiffness_c)
{
	if(stiffness_c.size() != SIZE_CART_STIFFNESS)
	{
		log(Error) << "setCartesianStiffness - expects a " << SIZE_CART_STIFFNESS << " dimensional vector" << endlog();
		return;
	}
	m_CartSpaceStiffness.data.assign(stiffness_c.begin(),stiffness_c.end());
}

void YouBot_RCC_executive::setJointStiffness(vector<double> stiffness_j)
{
	if(stiffness_j.size() != SIZE_JOINTS_ARRAY)
	{
		log(Error) << "setJointStiffness - expects a " << SIZE_JOINTS_ARRAY << " dimensional vector" << endlog();
		return;
	}
	m_JointSpaceStiffness.data.assign(stiffness_j.begin(),stiffness_j.end()); //Swap is valid since
}

void YouBot_RCC_executive::setJointsState(vector<double> position_j)
{
	if(position_j.size() != SIZE_JOINTS_ARRAY)
	{
		log(Error) << "positionArm - expects a " << SIZE_JOINTS_ARRAY << " dimensional vector" << endlog();
		return;
	}

	m_JointSpaceSetpoint.data.assign(position_j.begin(),position_j.end());
	stateTransition(JOINT_CONTROL);
}

void YouBot_RCC_executive::setHvp0(vector<double> position_c)
{
	if(position_c.size() != SIZE_H)
	{
		log(Error) << "positionGripper - expects a " << SIZE_H << " dimensional vector" << endlog();
		return;
	}

	m_Hvp0.data.assign(position_c.begin(),position_c.end());
	stateTransition(CARTESIAN_CONTROL);
}
void YouBot_RCC_executive::setHvptip(vector<double> position_c)
{
	if(position_c.size() != SIZE_H)
	{
		log(Error) << "positionGripper - expects a " << SIZE_H << " dimensional vector" << endlog();
		return;
	}
	log(Error) << "not implemented" << endlog();
			return;
	vector<double> t_Hvp0(16,0.0);
	MultiplyH(m_Htip0.data,position_c,t_Hvp0);
	m_Hvp0.data.assign(t_Hvp0.begin(),t_Hvp0.end());
	stateTransition(CARTESIAN_CONTROL);
}
void YouBot_RCC_executive::setHtipCC(vector<double> position_c)
{
	if(position_c.size() != SIZE_H)
	{
		log(Error) << "positionGripper - expects a " << SIZE_H << " dimensional vector" << endlog();
		return;
	}
	m_HtipCC.data.assign(position_c.begin(),position_c.end());
}

void YouBot_RCC_executive::getJointsState(vector<double> & position_j)
{
	if(position_j.size() != SIZE_JOINTS_ARRAY)
	{
		log(Error) << "getArmPose - expects a " << SIZE_JOINTS_ARRAY << " dimensional vector" << endlog();
		return;
	}

	position_j.assign(m_JointState.data.begin(), m_JointState.data.end());
}

void YouBot_RCC_executive::getTip_xyzypr(vector<double> & position_c)
{
	if(position_c.size() != SIZE_CART_SPACE)
	{
		log(Error) << "getGripperPose - expects a " << SIZE_CART_SPACE << " dimensional vector" << endlog();
		return;
	}
//	position_c.assign(m_Htip0.data.begin(),m_Htip0.data.end());
	homogeneous_to_xyzypr(m_Htip0.data, position_c);
}

void YouBot_RCC_executive::retractGripper()
{
	vector<double> _Hvptip(SIZE_H, 0.0);
	_Hvptip.assign(EYE4, EYE4+SIZE_H);
	_Hvptip[Z_H]=GRIPPER_SIZE;
	//displace center of compliance back to make movement more robust to disturbances
	//Test required if it produce good result
	setHtipCC(_Hvptip);
	m_CartSpaceStiffness.data.assign(RETRACT_STIFFNESS_C, RETRACT_STIFFNESS_C+SIZE_CART_STIFFNESS);
	setHvptip(_Hvptip);
}

void YouBot_RCC_executive::getHtip0(vector<double>& H)
{
	if(H.size() != SIZE_H)
	{
		log(Error) << "getGripperH - expects a "<<SIZE_H<<" dimensional vector" << endlog();
		return;
	}

	H.assign(m_Htip0.data.begin(), m_Htip0.data.end());
}

void YouBot_RCC_executive::guardMove(vector<double> force_c)
{
	if(force_c.size() != 3)
	{
		log(Error) << "guardMove - expects a 3 dimensional vector" << endlog();
		return;
	}

	m_force_cart = force_c;
	stateTransition(GUARDED_MOVE);
}

void YouBot_RCC_executive::doneEvent(){
	m_events = "executive.e_done";
	events.write(m_events);
}
bool YouBot_RCC_executive::isForceOverLimit_Norm()
{
	double sum_force;
	double sum_force_limit;
	sum_force=m_force_cart[0]*m_force_cart[0]+m_force_cart[1]*m_force_cart[1]+m_force_cart[2]*m_force_cart[2];
	sum_force_limit=m_Wtip0.data[3]*m_Wtip0.data[3]+m_Wtip0.data[4]*m_Wtip0.data[4]+m_Wtip0.data[5]*m_Wtip0.data[5];
	//squared values are always positive
	return sum_force<sum_force_limit;
}
bool YouBot_RCC_executive::isForceOverLimit_X()
{
	return abs(m_force_cart[0])<abs(m_Wtip0.data[3]);
}
bool YouBot_RCC_executive::isForceOverLimit_Y()
{
	return abs(m_force_cart[1])<abs(m_Wtip0.data[4]);
}
bool YouBot_RCC_executive::isForceOverLimit_Z()
{
	return abs(m_force_cart[2])<abs(m_Wtip0.data[5]);
}
void YouBot_RCC_executive::checkForceEvents(){
	if(!checkForce)
		return;
	if(isForceOverLimit_X())
	{
			m_events = "executive.e_CartForce_X_LIMIT_REACHED_true";
			events.write(m_events);
	}
	if(isForceOverLimit_Y())
	{
			m_events = "executive.e_CartForce_Y_LIMIT_REACHED_true";
			events.write(m_events);
	}
	if(isForceOverLimit_Z())
	{
			m_events = "executive.e_CartForce_Z_LIMIT_REACHED_true";
			events.write(m_events);
	}
	if(isForceOverLimit_Norm())
	{
			m_events = "executive.e_CartForce_LIMIT_REACHED_true";
			events.write(m_events);
	}

}


void YouBot_RCC_executive::stateTransition(state_t new_state)
{
		m_state = new_state;

}

void YouBot_RCC_executive::updateHook()
{
	// read all input ports
	Htip0.read(m_Htip0);
	JointsStates.read(m_JointState);
	Wtip0.read(m_Wtip0);

	// perform the state specific actions
	switch(m_state)
	{
		case(FULL_CONTROL):
		{
			stateFullControl();
			break;
		}
		case(GRAVITY_MODE):
		{
			stateGravityMode();
			break;
		}
		case(JOINT_CONTROL):
		{
			stateJointControl();
			break;
		}
		case(CARTESIAN_CONTROL):
		{
			stateCartesianControl();
			break;
		}
		case(GUARDED_MOVE):
		{
			stateGuardedMove();
			break;
		}
		default:
		{
			log(Error) << "control_state not recognized." << endlog();
			this->error();
			return;
			break;
		}
	}
	// write setpoints
if(m_JointSpaceSetpoint.data.size() == SIZE_JOINTS_ARRAY)
	JointSpaceSetpoint.write(m_JointSpaceSetpoint);
if(m_JointSpaceStiffness.data.size() == SIZE_JOINTS_ARRAY)
	JointSpaceStiffness.write(m_JointSpaceStiffness);
if(m_Hvp0.data.size() == SIZE_H)
	CartSpaceSetpoint.write(m_Hvp0);
if(m_CartSpaceStiffness.data.size() == SIZE_CART_STIFFNESS)
	CartSpaceStiffness.write(m_CartSpaceStiffness);
if(m_HtipCC.data.size() == SIZE_H)
	HtipCC.write(m_HtipCC);
}

void YouBot_RCC_executive::stateFullControl()
{
	//no additional actions required
}

void YouBot_RCC_executive::stateGravityMode()
{
	// Assigns the current states as setpoints and sets the stiffness zero
	// zero out Joint control part
	m_JointSpaceStiffness.data.assign(SIZE_JOINTS_ARRAY,0.0);
	m_JointSpaceSetpoint.data.assign(m_JointState.data.begin(), m_JointState.data.end());

	// zero out Cartesian control part
	m_CartSpaceStiffness.data.assign(SIZE_CART_STIFFNESS,0.0);
	m_Hvp0.data.assign(m_Htip0.data.begin(),m_Htip0.data.end());
}

void YouBot_RCC_executive::stateJointControl()
{
	// zero out Cartesian control part
	m_CartSpaceStiffness.data.assign(SIZE_CART_STIFFNESS,0.0);
	m_Hvp0.data.assign(m_Htip0.data.begin(),m_Htip0.data.end());
}

void YouBot_RCC_executive::stateCartesianControl()
{
	// zero out Joint control part
	m_JointSpaceStiffness.data.assign(SIZE_JOINTS_ARRAY,0.0);
	m_JointSpaceSetpoint.data.assign(m_JointState.data.begin(), m_JointState.data.end());
}


void YouBot_RCC_executive::stateGuardedMove()
{

	// Move with a predefined force in cartesian space
	vector<double> newHvp0(SIZE_H, 0.0);
	vector<double> newHvptip(SIZE_H, 0.0);
	newHvptip.assign(EYE4,EYE4+SIZE_H);
	newHvptip[X_H]=m_force_cart[0];
	newHvptip[Y_H]=m_force_cart[1];
	newHvptip[Z_H]=m_force_cart[2];
	vector<double> zero_stiffness_crt(SIZE_CART_STIFFNESS,0);
	if(  m_CartSpaceStiffness.data==zero_stiffness_crt)
	{
		log(Error)<<"Cartesian Stiffness is zero in all direction arm will not move;"<<endlog();
		log(Info)<<"Switching to gravity mode"<<endlog();
		gravityMode();
		return;
	}
	// Calculate forces
	if (isForceOverLimit_Norm())
	{
		m_events = "executive.e_CartForce_LIMIT_REACHED_true";
		events.write(m_events);
		m_Hvp0.data.assign(newHvp0.begin(),newHvp0.end());
		return;
	}
	else
	{

	MultiplyH(m_Htip0.data,newHvptip,newHvp0);
	m_Hvp0.data.assign(newHvp0.begin(),newHvp0.end());
	}
	// Joint space zero stiffness
	double zero_stiffness_jnt[SIZE_JOINTS_ARRAY] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	m_JointSpaceStiffness.data.assign(zero_stiffness_jnt, zero_stiffness_jnt + SIZE_JOINTS_ARRAY);
	//set joint set points to current state
	m_JointSpaceSetpoint.data.assign(m_JointState.data.begin(), m_JointState.data.end());


}

}

ORO_CREATE_COMPONENT( YouBot::YouBot_RCC_executive)



