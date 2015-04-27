#include "YouBotArmService.hpp"


namespace YouBot
{

extern unsigned int non_errors;

YouBotArmService::YouBotArmService(const string& name, TaskContext* parent, long i_clientID):
Hilas::IRobotArmService(name, parent, YouBot::NR_OF_ARM_SLAVES, 15, YouBot::JOINT_ARM_NAME_ARRAY,i_clientID), m_min_slave_nr(1)
{
    vrep_joint_handle.assign(YouBot::NR_OF_ARM_SLAVES, 0);

    is_in_visualization_mode = false;

    for (int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
    {
      simxGetObjectHandle(clientID, YouBot::JOINT_ARM_NAME_ARRAY[i].c_str(), &vrep_joint_handle[i], simx_opmode_oneshot_wait);
    }

    float p,v,e;
    for(int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
    {
      simxGetJointPosition(clientID, vrep_joint_handle[i], &p, simx_opmode_streaming);
      simxGetObjectFloatParameter(clientID, vrep_joint_handle[i], 2012, &v, simx_opmode_streaming);
      simxGetJointForce(clientID, vrep_joint_handle[i], &e, simx_opmode_streaming);
    }
}

YouBotArmService::~YouBotArmService(){}

void YouBotArmService::setControlModesAll(int mode)
{
  for(int i=0;i<YouBot::NR_OF_ARM_SLAVES;++i)
  {
    m_joint_ctrl_modes[i] = static_cast<Hilas::ctrl_modes>(mode);

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

void YouBotArmService::displayMotorStatuses(){}

void YouBotArmService::clearControllerTimeouts(){}

void YouBotArmService::readJointStates()
{
    float p,v,e;
    for(int i = 0; i < YouBot::NR_OF_ARM_SLAVES; ++i)
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

void YouBotArmService::updateJointSetpoints()
{
    // InputPort -> YouBot
    FlowStatus f = joint_position_command.read(m_joint_position_command);
    FlowStatus f1 = joint_velocity_command.read(m_joint_velocity_command);
    FlowStatus f2 = joint_effort_command.read(m_joint_effort_command);

    // Update joint setpoints
    simxPauseCommunication(clientID,1);
    for(unsigned int joint_nr = 0; joint_nr < YouBot::NR_OF_ARM_SLAVES; ++joint_nr)
    {
      assert(joint_nr < YouBot::NR_OF_ARM_SLAVES);

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
          simxSetJointTargetPosition(clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_oneshot);
          simxSetJointTargetPosition(clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_oneshot);
          simxSetJointTargetPosition(clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_oneshot);
          simxSetJointTargetPosition(clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_oneshot);
          simxSetJointTargetPosition(clientID,vrep_joint_handle[4],m_joint_state.position[4], simx_opmode_oneshot);
          simxSetJointTargetPosition(clientID,vrep_joint_handle[5],m_joint_state.position[5], simx_opmode_oneshot);                 
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

void YouBotArmService::checkMotorStatuses(){}

void YouBotArmService::update()
{
	if(is_in_visualization_mode)
	{
	  in_joint_state.read(m_joint_state);
	  simxSetJointPosition(clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_oneshot);
	  simxSetJointPosition(clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_oneshot);
	  simxSetJointPosition(clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_oneshot);
	  simxSetJointPosition(clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_oneshot);
	  simxSetJointPosition(clientID,vrep_joint_handle[4],m_joint_state.position[4], simx_opmode_oneshot);
	  simxSetJointPosition(clientID,vrep_joint_handle[5],m_joint_state.position[5], simx_opmode_oneshot);
	  return;
	}
	
	Hilas::IRobotArmService::update();
}

bool YouBotArmService::calibrate()
{
    log(Info) << "[VREP] Calibrating YouBotArmService" << endlog();
    if (m_calibrated)
    {
      log(Info) << "[VREP] Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "[VREP] Calibrated." << endlog();
    return (m_calibrated = true);
}

void YouBotArmService::stop(){}

void YouBotArmService::cleanup()
{
    m_calibrated = false;
}

}