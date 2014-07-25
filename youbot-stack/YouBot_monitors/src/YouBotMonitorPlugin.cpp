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
#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/types/GlobalsRepository.hpp>

#include <iostream>

#include "YouBotMonitorTypeInfos.hpp"

using namespace RTT;
using namespace std;

using namespace RTT;
using namespace RTT::types;
using namespace std;

using namespace YouBot;

std::string getRTTPluginName()
{
    return "YouBotMonitorPlugin";
}

/**
 * An example plugin which can be loaded in a process.
 * Orocos plugins should provide at least these two functions:
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    if ( t == 0 )
        cout << "Plugin of " << getRTTPluginName() << " loaded in process."<< endl;
    else
        cout << "Plugin of " << getRTTPluginName() << " loaded in component: "<< t->getName() << endl;

	RTT::types::Types()->addType( new ControlSpaceTypeInfo );
	RTT::types::Types()->addType( new PhysicalPartTypeInfo );
	RTT::types::Types()->addType( new PhysicalQuantityTypeInfo );
	RTT::types::Types()->addType( new EventTypeTypeInfo );
	RTT::types::Types()->addType( new CompareTypeTypeInfo );
	RTT::types::Types()->addType( new SequenceTypeInfo<std::vector<compare_type> >("std.vector<compare_type>") );

	GlobalsRepository::shared_ptr globals = GlobalsRepository::Instance();
	globals->setValue( new Constant<control_space>("JOINT",YouBot::JOINT) );
	globals->setValue( new Constant<control_space>("CARTESIAN",YouBot::CARTESIAN) );

	globals->setValue( new Constant<physical_part>("ARM",YouBot::ARM) );
	globals->setValue( new Constant<physical_part>("BASE",YouBot::BASE) );

	globals->setValue( new Constant<physical_quantity>("MONITOR_POSITION",YouBot::MONITOR_POSITION) );
	globals->setValue( new Constant<physical_quantity>("MONITOR_VELOCITY",YouBot::MONITOR_VELOCITY) );
	globals->setValue( new Constant<physical_quantity>("MONITOR_EFFORT",YouBot::MONITOR_EFFORT) );
	globals->setValue( new Constant<physical_quantity>("MONITOR_TIME",YouBot::MONITOR_TIME) );

	globals->setValue( new Constant<event_type>("EDGE",YouBot::EDGE) );
	globals->setValue( new Constant<event_type>("LEVEL",YouBot::LEVEL) );

	globals->setValue( new Constant<compare_type>("LESS",YouBot::LESS) );
	globals->setValue( new Constant<compare_type>("LESS_EQUAL",YouBot::LESS_EQUAL) );
	globals->setValue( new Constant<compare_type>("EQUAL",YouBot::EQUAL) );
	globals->setValue( new Constant<compare_type>("GREATER",YouBot::GREATER) );
	globals->setValue( new Constant<compare_type>("GREATER_EQUAL",YouBot::GREATER_EQUAL) );

    return true;
}
