#pragma once

#include <stddef.h>

namespace YouBot
{
	enum arm_settings {NR_OF_ARM_SLAVES=5};
	enum base_settings {NR_OF_BASE_SLAVES=4};

	enum ctrl_modes {PLANE_ANGLE = 1, ANGULAR_VELOCITY = 2 , TORQUE = 3, MOTOR_STOP = 4, TWIST = 5};

	const size_t max_event_length = 255;
}
