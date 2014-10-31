#include "YouBotBaseService.hpp"

#include <cassert>
#include <youbot/ProtocolDefinitions.hpp>
#include <tf_conversions/tf_kdl.h>

namespace YouBot
{
  using namespace RTT;
  using namespace RTT::types;
  using namespace std;

  YouBotBaseService::YouBotBaseService(const string& name, TaskContext* parent,
      unsigned int min_slave_nr, long i_clientID) :
      Service(name, parent), m_joint_ctrl_modes(NR_OF_BASE_SLAVES, MOTOR_STOP),
      // Set the commands to zero depending on the number of joints
      m_calibrated(false), m_min_slave_nr(min_slave_nr),
      m_clientID(i_clientID)
  {
    m_VREP = (YouBotSIM*) parent;

    vrep_joint_handle.assign(NR_OF_BASE_SLAVES,0);
    joint_base_position_prev.assign(4,0.0);
    odom_wheelPositions.assign(4,0.0*radian);

    is_in_visualization_mode = false;
    
    simxGetObjectHandle(m_clientID,"youBot",&all_robot_handle, simx_opmode_oneshot_wait);

    simxGetObjectHandle(m_clientID, "wheel_joint_fl", &vrep_joint_handle[0], simx_opmode_oneshot_wait);
    simxGetObjectHandle(m_clientID, "wheel_joint_fr", &vrep_joint_handle[1], simx_opmode_oneshot_wait);
    simxGetObjectHandle(m_clientID, "wheel_joint_bl", &vrep_joint_handle[2], simx_opmode_oneshot_wait);        
    simxGetObjectHandle(m_clientID, "wheel_joint_br", &vrep_joint_handle[3], simx_opmode_oneshot_wait);

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

    m_joint_state.position.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_state.name.push_back("wheel_joint_fl");
    m_joint_state.name.push_back("wheel_joint_fr");
    m_joint_state.name.push_back("wheel_joint_bl");
    m_joint_state.name.push_back("wheel_joint_br");
    m_joint_state.velocity.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_state.effort.assign(NR_OF_BASE_SLAVES, 0);

    m_joint_position_command.positions.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_velocity_command.velocities.assign(NR_OF_BASE_SLAVES, 0);
    m_joint_effort_command.efforts.assign(NR_OF_BASE_SLAVES, 0);

    // Pre-allocate port memory for outputs
    joint_state.setDataSample(m_joint_state);
    odometry_state.setDataSample(m_odometry_state);

    // Events - Pre-allocate port memory for outputs
    m_events.reserve(max_event_length);
    events.setDataSample(m_events);

    // odometry pose estimates frame
    m_odometry_state.header.frame_id = "odom";
    m_odometry_state.header.seq = 0;
    // odometry twist estimates frame
    m_odometry_state.child_frame_id = "base_footprint";

    // odometry estimates - set to zero
    m_odometry_state.pose.pose.position.x = 0;
    m_odometry_state.pose.pose.position.y = 0;
    m_odometry_state.pose.pose.position.z = 0;
    m_odometry_state.pose.pose.orientation.x = 0;
    m_odometry_state.pose.pose.orientation.y = 0;
    m_odometry_state.pose.pose.orientation.z = 0;
    m_odometry_state.pose.pose.orientation.w = 0;
    m_odometry_state.twist.twist.linear.x = 0;
    m_odometry_state.twist.twist.linear.y = 0;
    m_odometry_state.twist.twist.linear.z = 0;
    m_odometry_state.twist.twist.angular.x = 0;
    m_odometry_state.twist.twist.angular.y = 0;
    m_odometry_state.twist.twist.angular.z = 0;

    // set to false
    memset(m_overcurrent, 0, NR_OF_BASE_SLAVES);
    memset(m_undervoltage, 0, NR_OF_BASE_SLAVES);
    memset(m_overvoltage, 0, NR_OF_BASE_SLAVES);
    memset(m_overtemperature, 0, NR_OF_BASE_SLAVES);
    memset(m_connectionlost, 0, NR_OF_BASE_SLAVES);
    memset(m_i2texceeded, 0, NR_OF_BASE_SLAVES);
    memset(m_timeout, 0, NR_OF_BASE_SLAVES);

    // open data streaming from the simulator server
    float dummy_var;

    for(int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
	    simxGetJointPosition(m_clientID, vrep_joint_handle[i], &dummy_var, simx_opmode_streaming);
	    simxGetObjectFloatParameter(m_clientID, vrep_joint_handle[i], 2012, &dummy_var, simx_opmode_streaming);
	    simxGetJointForce(m_clientID, vrep_joint_handle[i], &dummy_var, simx_opmode_streaming); 
		}
    setupComponentInterface();
  }

  YouBotBaseService::~YouBotBaseService()
  {
    //delete m_base;
  }

  void YouBotBaseService::setsim_mode(int mode)
  {
    switch(mode)
    {
      case 1:
        is_in_visualization_mode = true;
      break;
      
      case 2:
        is_in_visualization_mode = false;
      break;
      
      default: break;
    }
  }  

  void YouBotBaseService::setupComponentInterface()
  {
    this->addPort("joint_state_out", joint_state).doc("Joint states");
    this->addPort("odometry_state_out", odometry_state).doc("Base odometry");

    this->addPort("joint_position_command_in", joint_position_command).doc("Command joint angles");
    this->addPort("joint_velocity_command_in", joint_velocity_command).doc("Command joint velocities");
    this->addPort("joint_effort_command_in", joint_effort_command).doc("Command joint torques");

    this->addPort("cmd_twist_in", cmd_twist).doc("Command base twist");

    this->addPort("joint_state_in", in_joint_state).doc("Joint states from HW or SIM");
    this->addPort("odometry_state_in", in_odometry_state).doc("Base odometry from HW or SIM");

    this->addPort("events", events).doc("Joint events");

    this->addOperation("start", &YouBotBaseService::start, this);
    this->addOperation("update", &YouBotBaseService::update, this);
    this->addOperation("calibrate", &YouBotBaseService::calibrate, this);
    this->addOperation("stop", &YouBotBaseService::stop, this);
    this->addOperation("cleanup", &YouBotBaseService::cleanup, this);
    this->addOperation("setsim_mode", &YouBotBaseService::setsim_mode, this);

	  this->addOperation("setControlModesAll", &YouBotBaseService::setControlModesAll,
		this, OwnThread).doc("Control modes can be set individually.");
    this->addOperation("setControlModes", &YouBotBaseService::setControlModes, this, OwnThread);
    this->addOperation("getControlModes", &YouBotBaseService::getControlModes, this, OwnThread);

    this->addOperation("displayMotorStatuses", &YouBotBaseService::displayMotorStatuses, this, OwnThread);
    this->addOperation("clearControllerTimeouts", &YouBotBaseService::clearControllerTimeouts, this, OwnThread);
  }

  void YouBotBaseService::getControlModes(vector<ctrl_modes>& all)
  {
    all = m_joint_ctrl_modes;
  }

  void YouBotBaseService::setControlModesAll(int mode)
  {
  	for(int i=0;i<NR_OF_BASE_SLAVES;++i)
    {
	  	m_joint_ctrl_modes[i] = static_cast<ctrl_modes>(mode);

      switch (m_joint_ctrl_modes[i])
      {
      case (PLANE_ANGLE):
      {
      	simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,1,simx_opmode_oneshot);
        break;
      }
      case (ANGULAR_VELOCITY):
      {
      	simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);
      	break;
      }
      case (TORQUE):
      {
      	simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);
        break;
      }
      case (MOTOR_STOP):
      {
      	simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,1,simx_opmode_oneshot);
        break;
      }
      case (TWIST):
      {
      	simxSetObjectIntParameter(m_clientID,vrep_joint_handle[i],VREP_JOINT_CONTROL_POSITION_IP,0,simx_opmode_oneshot);      	
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

  void YouBotBaseService::setControlModes(vector<ctrl_modes>& all)
  {
    if (all.size() != NR_OF_BASE_SLAVES)
    {
      log(Error) << "[VREP] The number of ctrl_modes should match the number of motors."
          << endlog();
      this->getOwner()->error();
      return;
    }

    // If one ctrl_mode is TWIST, check to see if all ctrl_modes are set to TWIST.
    bool twist(false);
    for (unsigned int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {
      if (all[i] == TWIST && !twist)
      {
        twist = true;
        if (i != 0)
        {
          this->getOwner()->error();
          log(Error) << "If the ctrl_mode TWIST is used, all "
              << NR_OF_BASE_SLAVES << " motors should be set to this!"
              << endlog();
          return;
        }
      }
      else if (twist && all[i] != TWIST)
      {
        this->getOwner()->error();
        log(Error) << "If the ctrl_mode TWIST is used, all " << NR_OF_BASE_SLAVES
            << " motors should be set to this!" << endlog();
        return;
      }
    }

    m_joint_ctrl_modes = all;
  }

  bool YouBotBaseService::start()
  {
    return m_calibrated;
  }

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

    simxPauseCommunication(m_clientID,1);

    for(unsigned int i=0; i < wheelVelocities.size() ; i++)
    {
      	simxSetJointTargetVelocity(m_clientID,vrep_joint_handle[i],wheelVelocities[i].value() * sign[i], simx_opmode_oneshot);
    }

    simxPauseCommunication(m_clientID,0);
  }

  void YouBotBaseService::setJointSetpoints()
  {
    joint_position_command.read(m_joint_position_command);
    joint_velocity_command.read(m_joint_velocity_command);
    joint_effort_command.read(m_joint_effort_command);

    FlowStatus f = joint_position_command.read(m_joint_position_command);
    FlowStatus f1 = joint_velocity_command.read(m_joint_velocity_command);
    FlowStatus f2 = joint_effort_command.read(m_joint_effort_command);    

    simxPauseCommunication(m_clientID,1);
    for (unsigned int joint_nr = 0; joint_nr < NR_OF_BASE_SLAVES; ++joint_nr)
    {
      switch (m_joint_ctrl_modes[joint_nr])
      {
      case (PLANE_ANGLE):
      {
        if(f != NewData) break;

        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[joint_nr],m_joint_position_command.positions[joint_nr], simx_opmode_oneshot);
        break;
      }
      case (ANGULAR_VELOCITY):
      {
        if(f1 != NewData) break;

        simxSetJointTargetVelocity(m_clientID,vrep_joint_handle[joint_nr],m_joint_velocity_command.velocities[joint_nr], simx_opmode_oneshot);
        break;
      }
      case (TORQUE):
      {
        if(f2 != NewData) break;

        simxSetJointForce(m_clientID,vrep_joint_handle[joint_nr],m_joint_effort_command.efforts[joint_nr], simx_opmode_oneshot);
        break;
      }
      case (MOTOR_STOP):
      {               
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_oneshot);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_oneshot);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_oneshot);
        simxSetJointTargetPosition(m_clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_oneshot);                
        break;
      }
      case (TWIST):
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
    simxPauseCommunication(m_clientID,0);    
  }

  void YouBotBaseService::readJointStates()
  {
    float p,v,e;

    for(int i = 0; i < NR_OF_BASE_SLAVES; ++i)
    {                
      simxGetJointPosition(m_clientID, vrep_joint_handle[i], &p, simx_opmode_buffer);
      simxGetObjectFloatParameter(m_clientID, vrep_joint_handle[i], 2012, &v, simx_opmode_buffer);
      simxGetJointForce(m_clientID, vrep_joint_handle[i], &e, simx_opmode_buffer);                  
 
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

    deltapos.assign(NR_OF_BASE_SLAVES,0);
    joints.assign(NR_OF_BASE_SLAVES,0);
    vels.assign(NR_OF_BASE_SLAVES,0);

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

      simxSetObjectPosition(m_clientID,all_robot_handle,-1,pos,simx_opmode_oneshot);

      double euler[3];
      simxFloat euler_s[3];

      //different coordinate systems, fixing it here
      Rotation orientation  = Rotation::Quaternion(
                          m_odometry_state.pose.pose.orientation.x,
                          m_odometry_state.pose.pose.orientation.y,
                          m_odometry_state.pose.pose.orientation.z,
                          m_odometry_state.pose.pose.orientation.w);

      // Instead of transforming for the inverse of this rotation,
      // we should find the right transform. (this is legacy code that
      // transforms vrep coords to rviz coords)
      Rotation rotation = Rotation::RPY(0,-M_PI/2,M_PI).Inverse();
      
      orientation = orientation * rotation;
      orientation.GetRPY(euler[0],euler[1],euler[2]); 

      euler_s[0] = euler[0];
      euler_s[1] = euler[1];
      euler_s[2] = euler[2];

      simxSetObjectOrientation(m_clientID,all_robot_handle,-1,euler_s,simx_opmode_oneshot);

      simxSetJointPosition(m_clientID,vrep_joint_handle[0],m_joint_state.position[0], simx_opmode_oneshot);
      simxSetJointPosition(m_clientID,vrep_joint_handle[1],m_joint_state.position[1], simx_opmode_oneshot);
      simxSetJointPosition(m_clientID,vrep_joint_handle[2],m_joint_state.position[2], simx_opmode_oneshot);
      simxSetJointPosition(m_clientID,vrep_joint_handle[3],m_joint_state.position[3], simx_opmode_oneshot);
      return;
    }

    readJointStates();
    readOdometry();

    if (m_joint_ctrl_modes[0] == TWIST) // All joints will be in TWIST ctrl_mode (see setControlModes)
    {
      setTwistSetpoints();
    }
    else
    {
      setJointSetpoints();
    }

    checkMotorStatuses();
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

  void YouBotBaseService::displayMotorStatuses(){}

  void YouBotBaseService::clearControllerTimeouts(){}
}
