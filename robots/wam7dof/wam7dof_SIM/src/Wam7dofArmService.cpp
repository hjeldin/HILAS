#include "Wam7dofArmService.hpp"


namespace Wam7dof
{

extern unsigned int non_errors;

Wam7dofArmService::Wam7dofArmService(const string& name, TaskContext* parent, long i_clientID):
Hilas::IRobotArmService(name, parent, Wam7dof::NR_OF_ARM_SLAVES, 6, Wam7dof::JOINT_ARM_NAME_ARRAY,i_clientID), m_min_slave_nr(1)
{
    vrep_joint_handle.assign(Wam7dof::NR_OF_ARM_SLAVES, 0);

    is_in_visualization_mode = false;

    for (int i = 0; i < Wam7dof::NR_OF_ARM_SLAVES; ++i)
    {
      log(Info) << "[SIM] Obtaining handle for arm joint " << Wam7dof::JOINT_ARM_NAME_ARRAY[i].c_str() << endlog();
      if(simxGetObjectHandle(clientID, Wam7dof::JOINT_ARM_NAME_ARRAY[i].c_str(), &vrep_joint_handle[i], simx_opmode_oneshot_wait) != simx_return_ok)
      {
        log(Error) << "Could not obtain arm joint" << endlog();
      }
    }

    float p,v,e;
    for(int i = 0; i < Wam7dof::NR_OF_ARM_SLAVES; ++i)
    {
      simxGetJointPosition(clientID, vrep_joint_handle[i], &p, simx_opmode_streaming);
      simxGetObjectFloatParameter(clientID, vrep_joint_handle[i], 2012, &v, simx_opmode_streaming);
      simxGetJointForce(clientID, vrep_joint_handle[i], &e, simx_opmode_streaming);
    }
}

Wam7dofArmService::~Wam7dofArmService(){}

void Wam7dofArmService::setControlModesAll(int mode)
{

  for(int i=0;i<Wam7dof::NR_OF_ARM_SLAVES;++i)
  {
    m_joint_ctrl_modes[i] = static_cast<Hilas::ctrl_modes>(mode);
     log(Info) << "[SIM] Set control mode to " << mode << " for joint " <<  Wam7dof::JOINT_ARM_NAME_ARRAY[i].c_str() << endlog();

      switch (m_joint_ctrl_modes[i])
      {
        case(Hilas::PLANE_ANGLE):
        {
          simxSetObjectIntParameter(clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,1,simx_opmode_oneshot);
          break;
        }
        case(Hilas::ANGULAR_VELOCITY):
        {
          simxSetObjectIntParameter(clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);
          break;
        }
        case(Hilas::TORQUE):
        {
          simxSetObjectIntParameter(clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);
          break;
        }
        case(Hilas::MOTOR_STOP):
        {
          simxSetObjectIntParameter(clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,1,simx_opmode_oneshot);
          break;
        }
        case(Hilas::TWIST):
        {
          log(Error) << "Case twist unable on arm." << endlog();
          this->getOwner()->error();
          break;
        }
        default:
        {
          log(Error) << "Case not recognized." << endlog();
          this->getOwner()->error();
          break;
        }
      }     
  }
}

void Wam7dofArmService::displayMotorStatuses(){}

void Wam7dofArmService::clearControllerTimeouts(){}

void Wam7dofArmService::readJointStates()
{
    float p,v,e;
    for(int i = 0; i < Wam7dof::NR_OF_ARM_SLAVES; ++i)
    {
      simxInt status = simxGetJointPosition(clientID, vrep_joint_handle[i], &p, simx_opmode_buffer);
      simxGetObjectFloatParameter(clientID, vrep_joint_handle[i], 2012, &v, simx_opmode_buffer);
      simxGetJointForce(clientID, vrep_joint_handle[i], &e, simx_opmode_buffer);

      m_joint_state.position[i] = p;
      m_joint_state.velocity[i] = v;
      m_joint_state.effort[i] = e;

    }

    joint_state.write(m_joint_state);
}

void Wam7dofArmService::updateJointSetpoints()
{
    // InputPort -> Wam7dof
    FlowStatus f = joint_position_command.read(m_joint_position_command);
    FlowStatus f1 = joint_velocity_command.read(m_joint_velocity_command);
    FlowStatus f2 = joint_effort_command.read(m_joint_effort_command);

    // Update joint setpoints
    simxPauseCommunication(clientID,1);
    for(unsigned int joint_nr = 0; joint_nr < Wam7dof::NR_OF_ARM_SLAVES; ++joint_nr)
    {
      assert(joint_nr < Wam7dof::NR_OF_ARM_SLAVES);

      switch (m_joint_ctrl_modes[joint_nr])
      {
        case(Hilas::PLANE_ANGLE):
        {
          //if(f != NewData) break;

          simxSetJointTargetPosition(clientID,vrep_joint_handle[joint_nr],m_joint_position_command.positions[joint_nr], simx_opmode_oneshot);
          break;
        }
        case(Hilas::ANGULAR_VELOCITY):
        {
          //if(f1 != NewData) break;
   
          simxSetJointTargetVelocity(clientID,vrep_joint_handle[joint_nr],m_joint_velocity_command.velocities[joint_nr], simx_opmode_oneshot);
          break;
        }
        case(Hilas::TORQUE):
        {
          //if(f2 != NewData) break;

          simxSetJointForce(clientID,vrep_joint_handle[joint_nr],m_joint_effort_command.efforts[joint_nr], simx_opmode_oneshot);
          break;
        }
        case(Hilas::MOTOR_STOP):
        {
          for (int i = 0; i < Wam7dof::NR_OF_ARM_SLAVES; ++i)
          {
            simxSetJointTargetPosition(clientID,vrep_joint_handle[i],m_joint_state.position[i], simx_opmode_oneshot);         
          }
          break;
        }
        default:
        {
          log(Error) << "ctrl_mode not recognized." << endlog();
          break;
        }
      }
    }
    simxPauseCommunication(clientID,0);
}

void Wam7dofArmService::checkMotorStatuses(){}

void Wam7dofArmService::update()
{
	if(is_in_visualization_mode)
	{
		in_joint_state.read(m_joint_state);
		for (int i = 0; i < Wam7dof::NR_OF_ARM_SLAVES; ++i)
        {
        	simxSetJointTargetPosition(clientID,vrep_joint_handle[i],m_joint_state.position[i], simx_opmode_oneshot);         
        }
		return;
	}
	
	Hilas::IRobotArmService::update();
}

bool Wam7dofArmService::calibrate()
{
    log(Info) << "[VREP] Calibrating Wam7dofArmService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "[VREP] Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "[VREP] Calibrated." << endlog();
    return (m_calibrated = true);
}

void Wam7dofArmService::stop(){}

void Wam7dofArmService::cleanup()
{
    m_calibrated = false;
}

}