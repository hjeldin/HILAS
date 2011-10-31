#pragma once

#include "YouBotMonitorTypes.hpp"

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

#include <sensor_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <geometry_msgs/typekit/Types.h>
#include <tf/tf.h>

#include <boost/function.hpp>
#include <vector>

namespace YouBot
{
	using namespace RTT;
	using namespace std;

	std::string compare_type_tostring(const compare_type& c_type);

	bool compare(const vector<unsigned int>& indices, const vector<double>& setp, const vector<double>& state, const compare_type ct, const double epsilon)
	{
		unsigned int index = 0;
		unsigned int size = indices.size();
		for(unsigned int i = 0; i < size; ++i)
		{
			index = indices[i];
			double diff = state[index] - setp[i];

//			log(Info) << compare_type_tostring(ct) << " diff (" << cur[index] << " - " << (*setp)[index] << ") : " << diff << " epsilon: " << epsilon << endlog();

			if( ct == LESS && diff >= 0 )
			{
				return false;
			}
			else if(ct == LESS_EQUAL && diff > 0 )
			{
				return false;
			}
			else if(ct == EQUAL)
			{
				double relativeError = 0.0;
				if(setp[i] == state[index])
					continue; // true
				else if(abs(setp[i]) > abs(state[index]))
					relativeError = abs( (state[index] - setp[i]) / setp[i]);
				else
					relativeError = abs( (state[index] - setp[i]) / state[index]);

				if(relativeError > epsilon)
				{
//					log(Info) << "__false__" << endlog();
					return false;
				}
			}
			else if(ct == GREATER_EQUAL && diff < 0 )
			{
				return false;
			}
			else if(ct == GREATER && diff <= 0 )
			{
				return false;
			}
		}

		return (size > 0) ? true : false;
	}

	void homogeneous_to_xyzypr(std::vector<double> H, std::vector<double> xyzypr)
	{
		assert(H.size() == 16 && xyzypr.size() == 6);

		xyzypr[0] = H[3];
		xyzypr[1] = H[7];
		xyzypr[2] = H[11];

		btMatrix3x3 rotMatrix(H[0], H[1], H[2], H[4], H[5], H[6], H[8], H[9], H[10]);

		btScalar y, p, r;
		rotMatrix.getEulerYPR(y, p, r);
		xyzypr[3] = y;
		xyzypr[4] = p;
		xyzypr[5] = r;
	}

	std::string control_space_tostring(const control_space& space)
	{
		if(space == CARTESIAN)
			return "CARTESIAN";
		else if(space == JOINT)
			return "JOINT";
		else
			return "error";
	}

	std::string control_space_toeventstring(const control_space& space)
	{
		if(space == CARTESIAN)
			return "cart";
		else if(space == JOINT)
			return "jnt";
		else
			return "error";
	}

	std::string physical_quantity_tostring(const physical_quantity& quantity)
	{
		if(quantity == MONITOR_POSITION)
			return "MONITOR_POSITION";
		else if(quantity == MONITOR_VELOCITY)
			return "MONITOR_VELOCITY";
		else if(quantity == MONITOR_FORCE)
			return "MONITOR_FORCE";
		else if(quantity == MONITOR_TORQUE)
			return "MONITOR_TORQUE";
		else if(quantity == MONITOR_TIME)
			return "MONITOR_TIME";
		else
			return "error";
	}

	std::string physical_quantity_toeventstring(const physical_quantity& quantity)
	{
		if(quantity == MONITOR_POSITION)
			return "pos";
		else if(quantity == MONITOR_VELOCITY)
			return "vel";
		else if(quantity == MONITOR_FORCE)
			return "for";
		else if(quantity == MONITOR_TORQUE)
			return "tor";
		else if(quantity == MONITOR_TIME)
			return "time";
		else
			return "error";
	}

	std::string physical_part_tostring(const physical_part& part)
	{
		if(part == ARM)
			return "ARM";
		else if(part == BASE)
			return "BASE";
		else
			return "error";
	}

	std::string event_type_tostring(const event_type& e_type)
	{
		if(e_type == EDGE)
			return "EDGE";
		else if(e_type == LEVEL)
			return "LEVEL";
		else
			return "error";
	}

	std::string compare_type_tostring(const compare_type& c_type)
	{
		if(c_type == LESS)
			return "LESS";
		else if(c_type == LESS_EQUAL)
			return "LESS_EQUAL";
		else if(c_type == EQUAL)
			return "EQUAL";
		else if(c_type == GREATER)
			return "GREATER";
		else if(c_type == GREATER_EQUAL)
			return "GREATER_EQUAL";
		else
			return "error";
	}

}
