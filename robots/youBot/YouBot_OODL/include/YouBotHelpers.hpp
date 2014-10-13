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

#pragma once

#include <vector>
#include <iostream>

#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/RTT.hpp>

#include "YouBotTypes.hpp"

namespace RTT
{
	namespace types
	{
		using namespace YouBot;

		std::ostream& operator<<(std::ostream& os, const ctrl_modes& cd);

		std::ostream& operator<<(std::ostream& os, const std::vector<ctrl_modes>& cd);

		std::istream& operator>>(std::istream& is, std::vector<ctrl_modes>& cd);

		std::istream& operator>>(std::istream& is, ctrl_modes& cd);
	}
}

namespace YouBot
{
	static const std::string E_OVERCURRENT = "e_OVERCURRENT";
	static const std::string E_UNDERVOLTAGE = "e_UNDERVOLTAGE";
	static const std::string E_OVERVOLTAGE = "e_OVERVOLTAGE";
	static const std::string E_OVERTEMP = "e_OVERTEMP";
	static const std::string E_EC_CONN_LOST = "e_EC_CONN_LOST";
	static const std::string E_I2T_EXCEEDED = "e_I2T_EXCEEDED";
	static const std::string E_HALL_ERR = "e_HALL_ERR";
	static const std::string E_ENCODER_ERR = "e_ENCODER_ERR";
	static const std::string E_SINE_COMM_INIT_ERR = "e_SINE_COMM_INIT_ERR";
	static const std::string E_EMERGENCY_STOP = "e_EMERGENCY_STOP";
	static const std::string E_EC_TIMEOUT = "e_EC_TIMEOUT";

	std::string& make_event(std::string& s, const std::string& event, int num);
	std::string& make_edge_event(std::string& s, const std::string& event, int num, bool status);

	std::string ctrl_modes_tostring(const ctrl_modes& cd);

	std::string motor_status_tostring(const unsigned int& cd);

	struct CtrlModesTypeInfo : public RTT::types::TemplateTypeInfo<ctrl_modes, true>
	{
		CtrlModesTypeInfo();

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const ctrl_modes& in, RTT::PropertyBag& targetbag ) const;

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, ctrl_modes& out ) const;
	};
}

//----------------
// Helper macro's for YouBotArmService and YouBotBaseService
//----------------

#define CHECK_EVENT_EDGE(OODL_EVENT, COND_STORAGE, OUTPUT_MSG) \
    if((tmp & OODL_EVENT) != 0 && !(COND_STORAGE[joint]) ) \
    { \
      COND_STORAGE[joint] = true; \
      events.write(make_edge_event(m_events, OUTPUT_MSG, joint+1, true)); \
    } \
    else if(COND_STORAGE[joint] && (tmp & OODL_EVENT) == 0) \
    { \
      COND_STORAGE[joint] = false; \
      events.write(make_edge_event(m_events, OUTPUT_MSG, joint+1, false)); \
    }

#define CHECK_EVENT_LEVEL(OODL_EVENT, OUTPUT_MSG) \
  if((tmp & OODL_EVENT) != 0) \
  { \
    events.write(make_event(m_events, OUTPUT_MSG, joint+1)); \
  }
