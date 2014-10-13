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

#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <iostream>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <YouBotTypes.hpp>
#include "YouBotHelpers.hpp"

using namespace RTT;
using namespace RTT::types;
using namespace std;
/**
 * An example plugin which can be loaded in a process.
 * Orocos plugins should provide at least these two functions:
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    if ( t == 0 )
        cout << "Plugin of YouBot helpers loaded in process."<< endl;
    else
        cout << "Plugin of YouBot helpers in component: "<< t->getName() << endl;

	RTT::types::Types()->addType( new CtrlModesTypeInfo );
	RTT::types::Types()->addType( new SequenceTypeInfo<std::vector<ctrl_modes> >("std.vector<ctrl_modes>") );

	GlobalsRepository::shared_ptr globals = GlobalsRepository::Instance();
	globals->setValue( new Constant<ctrl_modes>("PLANE_ANGLE",YouBot::PLANE_ANGLE) );
	globals->setValue( new Constant<ctrl_modes>("ANGULAR_VELOCITY",YouBot::ANGULAR_VELOCITY) );
	globals->setValue( new Constant<ctrl_modes>("TORQUE",YouBot::TORQUE) );
	globals->setValue( new Constant<ctrl_modes>("TWIST",YouBot::TWIST) );
	globals->setValue( new Constant<ctrl_modes>("MOTOR_STOP",YouBot::MOTOR_STOP) );
    return true;
}

std::string getRTTPluginName()
{
    return "YouBotHelpersPlugin";
}
