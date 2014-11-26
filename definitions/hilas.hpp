#pragma once

#define TASK_SPACE_SIZE 6
#define VREP_JOINT_CONTROL_POSITION_IP 2001

namespace Hilas
{
enum ctrl_modes{PLANE_ANGLE = 1, ANGULAR_VELOCITY = 2 , TORQUE = 3, MOTOR_STOP = 4, TWIST = 5};
const size_t max_event_length = 255;

typedef struct _JointLimits
{
    double min_angle;
    double max_angle;

    _JointLimits(): min_angle(0.0), max_angle(0.0){}
}JointLimits;

typedef struct _GripperJointLimits
{
    double min_position;
    double max_position;

    _GripperJointLimits(): min_position(0.0), max_position(0.0){}
}GripperJointLimits;
}