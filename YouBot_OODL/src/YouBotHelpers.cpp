/**********************************************************************
 *
 * Copyright (c) 2010-2013
 * All rights reserved.
 *
 * Robotics and Mechatronics (RaM) group
 * Faculty of Electrical Engineering, Mathematics and Computer Science
 * University of Twente
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author(s):
 * Robert Wilterdink, Yury Brodskiy 
 *
 * Supervised by: 
 * Jan F. Broenink
 * 
 * The research leading to these results has received funding from the 
 * European Community's Seventh Framework Programme (FP7/2007-2013) 
 * under grant agreement no. FP7-ICT-231940-BRICS (Best Practice in 
 * Robotics).
 * 
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This software is published under a dual-license: GNU Lesser General 
 * Public License LGPL 2.1 and BSD license. The dual-license implies 
 * that users of this code may choose which terms they prefer.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above 
 *       copyright notice, this list of conditions and the following 
 *       disclaimer in the documentation and/or other materials 
 *       provided with the distribution.
 *     * Neither the name of the University of Twente nor the names of 
 *       its contributors may be used to endorse or promote products 
 *       derived from this software without specific prior written 
 *       permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more 
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 **********************************************************************/

#include "YouBotHelpers.hpp"

#include <rtt/Logger.hpp>
#include <ostream>
#include <istream>
#include <boost/algorithm/string.hpp>
#include <youbot/ProtocolDefinitions.hpp>

namespace RTT
{
	namespace types
	{
		using namespace YouBot;
		using namespace RTT;

		std::ostream& operator<<(std::ostream& os, const ctrl_modes& cd) {
			return os << ctrl_modes_tostring(cd);
		}

		std::ostream& operator<<(std::ostream& os, const std::vector<ctrl_modes>& cd) {
			for(unsigned int i = 0; i < cd.size(); ++i)
			{
				os << cd[i];
			}
			return os;
		}

		std::istream& operator>>(std::istream& is, std::vector<ctrl_modes>& cd) {
			char c;
			for(unsigned int i = 0; i < cd.size(); ++i)
			{
				is >> c >> cd[i];
			}
			return is >> c;
		}

		std::istream& operator>>(std::istream& is, ctrl_modes& cd) {
			return is >> cd;
		}
	}
}

namespace YouBot
{

	using namespace RTT;

  // real-time safe provided s has been resized to max_event_strlen
	//@source: KU Leuven, youbot_helpers.cpp in youbot_master_rtt.
  std::string& make_event(std::string& s, const std::string& event, int num)
  {
      char tmpstr[max_event_length];
      if(s.capacity() < max_event_length)
          log(Error) << "make_event: event string capacity < max_event_length." << endlog();

      snprintf(tmpstr, max_event_length, "%s,jointid:%d", event.c_str(), num);
      s.insert(0, tmpstr, max_event_length);
      return s;
  }

  std::string& make_edge_event(std::string& s, const std::string& event, int num, bool status)
  {
      char tmpstr[max_event_length];
      if(s.capacity() < max_event_length)
          log(Error) << "make_event: event string capacity < max_event_length." << endlog();

      if(status == true)
        snprintf(tmpstr, max_event_length, "%s,jointid:%d,true", event.c_str(), num);
      else
        snprintf(tmpstr, max_event_length, "%s,jointid:%d,false", event.c_str(), num);

      s.insert(0, tmpstr, max_event_length);
      return s;
  }

	std::string ctrl_modes_tostring(const ctrl_modes& cd)
	{
		switch(cd)
		{
			case(PLANE_ANGLE):
				return "PLANE_ANGLE";
				break;
			case(ANGULAR_VELOCITY):
				return "ANGULAR_VELOCITY";
				break;
			case(TORQUE):
				return "TORQUE";
				break;
			case(MOTOR_STOP):
				return "MOTOR_STOP";
				break;
			case(TWIST):
			  return "TWIST";
			  break;
			default:
				log(Error) << "Control mode not recognized." << endlog();
				return "UNKNOWN";
		}
	}

	std::string motor_status_tostring(const unsigned int& cd)
	{
    using namespace youbot;

		std::stringstream errors;

		if (cd & OVER_CURRENT) {
		  errors << "OVER_CURRENT ";
		}

		if (cd & UNDER_VOLTAGE) {
		  errors << "UNDER_VOLTAGE ";
		}

		if (cd & OVER_VOLTAGE) {
		  errors << "OVER_VOLTAGE ";
		}

		if (cd & OVER_TEMPERATURE) {
		  errors << "OVER_TEMPERATURE ";
		}

		if (cd & MOTOR_HALTED) {
		  errors << "MOTOR_HALTED ";
		}

		if (cd & HALL_SENSOR_ERROR) {
		  errors << "HALL_SENSOR_ERROR ";
		}

	//    if (messageBuffer.stctInput.errorFlags & ENCODER_ERROR) {
	//      statusMessages.push_back(errorMessage + "got encoder problem");
	//    }
	//
	//     if (messageBuffer.stctInput.errorFlags & INITIALIZATION_ERROR) {
	//      statusMessages.push_back(errorMessage + "got inizialization problem");
	//    }

//		if (cd & PWM_MODE_ACTIVE) {
//		  errors << "PWM_MODE_ACTIVE ";
//		}

		if (cd & VELOCITY_MODE) {
		  errors << "VELOCITY_MODE ";
		}

		if (cd & POSITION_MODE) {
		  errors << "POSITION_MODE ";
		}

		if (cd & TORQUE_MODE) {
		  errors << "TORQUE_MODE ";
		}

	//    if (messageBuffer.stctInput.errorFlags & EMERGENCY_STOP) {
	//      statusMessages.push_back(errorMessage + "has emergency stop active");
	//    }
	//
	//    if (messageBuffer.stctInput.errorFlags & FREERUNNING) {
	//      statusMessages.push_back(errorMessage + "has freerunning active");
	//    }

		if (cd & POSITION_REACHED) {
		  errors << "POSITION_REACHED ";
		}

		if (cd & INITIALIZED) {
		  errors << "INITIALIZED ";
		}

		if (cd & TIMEOUT) {
		  errors << "TIMEOUT ";
		}

		if (cd & I2T_EXCEEDED) {
		  errors << "I2T_EXCEEDED ";
		}

		if(errors.str() == "")
		{
			errors << "OK";
		}

		return errors.str();
	}

	CtrlModesTypeInfo::CtrlModesTypeInfo() : RTT::types::TemplateTypeInfo<ctrl_modes, true>( "ctrl_modes" )
	{ }

	// this is a helper function, which is called by composeType() of the same class:
	bool CtrlModesTypeInfo::decomposeTypeImpl(const ctrl_modes& in, RTT::PropertyBag& targetbag ) const {
		log(Error) << "Not implemented!" << endlog();
		return false;
	}

	bool CtrlModesTypeInfo::composeTypeImpl(const RTT::PropertyBag& bag, ctrl_modes& out ) const
	{
		log(Error) << "Not implemented!" << endlog();
		return false; // unknown type !
	}

}
