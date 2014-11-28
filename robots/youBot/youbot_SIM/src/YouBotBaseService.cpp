#include "YouBotBaseService.hpp"

namespace YouBot
{

  YouBotBaseService::YouBotBaseService(const string& name, TaskContext* parent, long i_clientID):
  Hilas::IRobotBaseService(name, parent, YouBot::NR_OF_BASE_SLAVES, 15, YouBot::JOINT_BASE_NAME_ARRAY,i_clientID), m_min_slave_nr(1)
  {
    vrep_joint_handle.assign(YouBot::NR_OF_BASE_SLAVES,0);
    joint_base_position_prev.assign(4,0.0);
    odom_wheelPositions.assign(4,0.0*radian);

    is_in_visualization_mode = false;
    
    simxGetObjectHandle(clientID,"youBot",&all_robot_handle, simx_opmode_oneshot_wait);

    for (int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
    {
      simxGetObjectHandle(clientID, YouBot::JOINT_BASE_NAME_ARRAY[i].c_str(), &vrep_joint_handle[i], simx_opmode_oneshot_wait);
    }

    configfile.reset(new youbot::ConfigFile("youbot-base.cfg",CFG_YOUBOT_BASE));
    configfile->readInto(kinematicConfig.rotationRatio, "YouBotKinematic", "RotationRatio");   
    configfile->readInto(kinematicConfig.slideRatio, "YouBotKinematic", "SlideRatio");

    double dummy = 0;
    
    configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontAndRearWheels_[meter]");
    kinematicConfig.lengthBetweenFrontAndRearWheels = dummy * meter;
    configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontWheels_[meter]");
    kinematicConfig.lengthBetweenFrontWheels = dummy * meter;
    configfile->readInto(dummy, "YouBotKinematic", "WheelRadius_[meter]");
    kinematicConfig.wheelRadius = dummy * meter;
    youBotBaseKinematic.setConfiguration(kinematicConfig);

    // open data streaming from the simulator server
    float dummy_var;

    for(int i = 0; i < YouBot::NR_OF_BASE_SLAVES; ++i)
    {
      simxGetJointPosition(clientID, vrep_joint_handle[i], &dummy_var, simx_opmode_streaming);
      simxGetObjectFloatParameter(clientID, vrep_joint_handle[i], 2012, &dummy_var, simx_opmode_streaming);
      simxGetJointForce(clientID, vrep_joint_handle[i], &dummy_var, simx_opmode_streaming); 
    }    

  }

  YouBotBaseService::~YouBotBaseService(){}

  void YouBotBaseService::setControlModesAll(int mode)
  {
    for(int i=0;i<YouBot::NR_OF_BASE_SLAVES;++i)
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
        simxSetObjectIntParameter(clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);        
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

  void YouBotBaseService::displayMotorStatuses(){}

  void YouBotBaseService::clearControllerTimeouts(){}  

  void YouBotBaseService::setTwistSetpoints()
  {
    cmd_twist.read(m_cmd_twist);
    std::vector<quantity<angular_velocity> > wheelVelocities;

    quantity<si::velocity> longitudinalVelocity;
    quantity<si::velocity> transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;

    longitudinalVelocity = m_cmd_twist.linear.x * meter_per_second;
    transversalVelocity = m_cmd_twist.linear.y * meter_per_second;
    angularVelocity = m_cmd_twist.angular.z * radian_per_second;

    youBotBaseKinematic.cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, wheelVelocities);

    // Little hack
    int sign[4] = {1,-1,1,-1};

    simxPauseCommunication(clientID,1);

    for(unsigned int i=0; i < wheelVelocities.size() ; i++)
    {
        simxSetJointTargetVelocity(clientID,vrep_joint_handle[i],wheelVelocities[i].value() * sign[i], simx_opmode_oneshot);
    }

    simxPauseCommunication(clientID,0);
  }

  void YouBotBaseService::setJointSetpoints()
  {
    joint_position_command.read(m_joint_position_command);
    joint_velocity_command.read(m_joint_velocity_command);
    joint_effort_command.read(m_joint_effort_command);

    FlowStatus f = joint_position_command.read(m_joint_position_command);
    FlowStatus f1 = joint_velocity_command.read(m_joint_velocity_command);
    FlowStatus f2 = joint_effort_command.read(m_joint_effort_command);    

    simxPauseCommunication(clientID,1);
    for (unsigned int joint_nr = 0; joint_nr < YouBot::NR_OF_BASE_SLAVES; ++joint_nr)
    {
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
        
        // Little hack
        int sign[4] = {1,-1,1,-1};
        simxSetJointTargetVelocity(clientID,vrep_joint_handle[joint_nr],m_joint_velocity_command.velocities[joint_nr] * sign[joint_nr], simx_opmode_oneshot);
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
        break;
      }
      case(Hilas::TWIST):
      {
        log(Error) << "Cannot be in TWIST ctrl_mode (programming error)"
            << endlog();
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
    simxPauseCommunication(clientID,0);
  }

 void YouBotBaseService::readJointStates()
  {
    float p,v,e;

    for(int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {                
      simxGetJointPosition(clientID, vrep_joint_handle[i], &p, simx_opmode_buffer);
      simxGetObjectFloatParameter(clientID, vrep_joint_handle[i], 2012, &v, simx_opmode_buffer);
      simxGetJointForce(clientID, vrep_joint_handle[i], &e, simx_opmode_buffer);                  
 
      m_joint_state.position[i] = p;
      m_joint_state.velocity[i] = v;
      m_joint_state.effort[i] = e;
    }
    
    joint_state.write(m_joint_state);    
  }
  
  void YouBotBaseService::readOdometry()
  {
    /* Weird code! Don't touch it, here be dragons. */

    std::vector<double> deltapos;
    std::vector<double> joints;
    std::vector<double> vels;

    quantity<si::length> longitudinalPosition;
    quantity<si::length> transversalPosition;
    quantity<plane_angle> orientation;

    deltapos.assign(YouBot::NR_OF_BASE_SLAVES,0);
    joints.assign(YouBot::NR_OF_BASE_SLAVES,0);
    vels.assign(YouBot::NR_OF_BASE_SLAVES,0);

    int sign[4]={1,-1,-1,1};

    /* ----------------------MAGIC---------------------------- */

    joints[0] = m_joint_state.position[3] * sign[0];
    joints[1] = m_joint_state.position[2] * sign[1];
    joints[2] = m_joint_state.position[0] * sign[2];
    joints[3] = m_joint_state.position[1] * sign[3];        

    vels[0] = -m_joint_state.velocity[3];
    vels[1] = m_joint_state.velocity[2];
    vels[2] = -m_joint_state.velocity[0];
    vels[3] = m_joint_state.velocity[1];

    /* -------------------------------------------------------- */

    double long_vel, trans_vel, ang_vel;
    double geom_factor = (kinematicConfig.lengthBetweenFrontAndRearWheels.value() / 2.0) + (kinematicConfig.lengthBetweenFrontWheels.value() / 2.0);

    long_vel = (-vels[0] + vels[1] - vels[2] + vels[3]) * (kinematicConfig.wheelRadius.value() / 4);
    trans_vel = (vels[0] + vels[1] - vels[2] - vels[3]) * (kinematicConfig.wheelRadius.value() / 4);
    ang_vel = (vels[0] + vels[1] + vels[2] + vels[3]) * ((kinematicConfig.wheelRadius.value() / 4) / geom_factor);

    deltapos[0] =  joints[1] - joint_base_position_prev[0];
    deltapos[1] =  joints[0] - joint_base_position_prev[1];
    deltapos[2] =  joints[2] - joint_base_position_prev[2];
    deltapos[3] =  joints[3] - joint_base_position_prev[3];

    for(int i=0; i < 4; ++i)
    {
      if(deltapos[i] > M_PI)
      {
        deltapos[i] += -M_PI * 2;
      }
      else if(deltapos[i] < -M_PI)
      {
        deltapos[i] += 2 * M_PI;        
      }
    }

    for(int i=0; i < 4; i++)
    {
      odom_wheelPositions[i] += deltapos[i] * radian;
    }

    joint_base_position_prev[0] = joints[1];
    joint_base_position_prev[1] = joints[0];
    joint_base_position_prev[2] = joints[2];
    joint_base_position_prev[3] = joints[3];

    youBotBaseKinematic.wheelPositionsToCartesianPosition(odom_wheelPositions, longitudinalPosition, transversalPosition, orientation);

    m_odometry_state.pose.pose.position.x = -longitudinalPosition.value();
    m_odometry_state.pose.pose.position.y = transversalPosition.value();
    m_odometry_state.pose.pose.position.z = 0.095;

    m_odometry_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-orientation.value());

    m_odometry_state.twist.twist.linear.x = long_vel;
    m_odometry_state.twist.twist.linear.y = trans_vel;
    m_odometry_state.twist.twist.linear.z = 0.0;
    m_odometry_state.twist.twist.angular.x = 0.0;
    m_odometry_state.twist.twist.angular.y = 0.0;
    m_odometry_state.twist.twist.angular.z = -ang_vel;

    odometry_state.write(m_odometry_state);
  }

  void YouBotBaseService::update()
  {
    if(is_in_visualization_mode)
    {
      in_odometry_state.read(m_odometry_state);

      simxFloat pos[3];
      pos[0] = m_odometry_state.pose.pose.position.x;
      pos[1] = m_odometry_state.pose.pose.position.y;
      pos[2] = m_odometry_state.pose.pose.position.z;

      simxSetObjectPosition(clientID,all_robot_handle,-1,pos,simx_opmode_oneshot);

      double euler[3];
      simxFloat euler_s[3];

      //different coordinate systems, fixing it here
      KDL::Rotation orientation  = KDL::Rotation::Quaternion(
                          m_odometry_state.pose.pose.orientation.x,
                          m_odometry_state.pose.pose.orientation.y,
                          m_odometry_state.pose.pose.orientation.z,
                          m_odometry_state.pose.pose.orientation.w);

      // Instead of transforming for the inverse of this rotation,
      // we should find the right transform. (this is legacy code that
      // transforms vrep coords to rviz coords)
      KDL::Rotation rotation = KDL::Rotation::RPY(0,-M_PI/2,M_PI).Inverse();
      
      orientation = orientation * rotation;
      orientation.GetRPY(euler[0],euler[1],euler[2]); 

      euler_s[0] = euler[0];
      euler_s[1] = euler[1];
      euler_s[2] = euler[2];

      simxSetObjectOrientation(clientID,all_robot_handle,-1,euler_s,simx_opmode_oneshot);

      simxSetJointPosition(clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_oneshot);
      simxSetJointPosition(clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_oneshot);
      simxSetJointPosition(clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_oneshot);
      simxSetJointPosition(clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_oneshot);
      return;
    }

      Hilas::IRobotBaseService::update();
  }


  void YouBotBaseService::checkMotorStatuses(){}

  bool YouBotBaseService::calibrate()
  {
    if (m_calibrated)
    {
      log(Info) << "Already calibrated." << endlog();
      return m_calibrated;
    }

    log(Info) << "[VREP] Calibrating YouBotBaseService..." << endlog();

    log(Info) << "[VREP] Calibrated." << endlog();
    return (m_calibrated = true);
  }

  void YouBotBaseService::stop(){}

  void YouBotBaseService::cleanup()
  {
    m_calibrated = false;
  }
}