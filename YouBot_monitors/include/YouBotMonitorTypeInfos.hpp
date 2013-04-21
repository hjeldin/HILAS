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

#include "YouBotMonitorHelpers.hpp"

#include <vector>
#include <iostream>

#include <boost/algorithm/string.hpp>

#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace RTT
{
	namespace types
	{
		using namespace YouBot;

		std::ostream& operator<<(std::ostream& os, const control_space& cd) {
			return os << control_space_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, control_space& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const physical_quantity& cd) {
			return os << physical_quantity_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, physical_quantity& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const physical_part& cd) {
			return os << physical_part_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, physical_part& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const event_type& cd) {
			return os << event_type_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, event_type& cd) {
			return is >> cd;
		}

		std::ostream& operator<<(std::ostream& os, const compare_type& cd) {
			return os << compare_type_tostring(cd);
		}

		std::istream& operator>>(std::istream& is, compare_type& cd) {
			return is >> cd;
		}
	}
}

// UGLY: How to do this properly?

namespace YouBot
{
	using namespace RTT;

	struct ControlSpaceTypeInfo : public RTT::types::TemplateTypeInfo<control_space, true>
	{
		ControlSpaceTypeInfo(): RTT::types::TemplateTypeInfo<control_space, true>( "control_space" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const control_space& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("control_space");
			targetbag.add( new Property<std::string>("control_space", "", control_space_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, control_space& out ) const
		{
			if ( bag.getType() == std::string("control_space") ) // check the type
			{
				Property<std::string>* cs = dynamic_cast<Property<std::string>*>(bag.getProperty("control_space"));

				if ( !cs )
					return false;

				std::string str = cs->get();
				if(boost::equals(str, "JOINT"))
				{
					out = JOINT;
				}
				else if(boost::equals(str, "CARTESIAN"))
				{
					out = CARTESIAN;
				}
				else
				{
					log(Error) << "Enum value (control_space) not recognized." << endlog();
					return false;
				}

				return true;
			}
			else
			{
				return false; // unknown type !
			}
		}
	};

	struct PhysicalPartTypeInfo : public RTT::types::TemplateTypeInfo<physical_part, true>
	{
		PhysicalPartTypeInfo(): RTT::types::TemplateTypeInfo<physical_part, true>( "physical_part" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const physical_part& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("physical_part");
			targetbag.add( new Property<std::string>("physical_part", "", physical_part_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, physical_part& out ) const
		{
			if ( bag.getType() == std::string("physical_part") ) // check the type
			{
				Property<std::string>* cs = dynamic_cast<Property<std::string>*>(bag.getProperty("physical_part"));

				if ( !cs )
					return false;

				std::string str = cs->get();
				if(boost::equals(str, "ARM"))
				{
					out = ARM;
				}
				else if(boost::equals(str, "BASE"))
				{
					out = BASE;
				}
				else
				{
					log(Error) << "Enum value (physical_part) not recognized." << endlog();
					return false;
				}

				return true;
			}
			else
			{
				return false; // unknown type !
			}
		}
	};

	struct PhysicalQuantityTypeInfo : public RTT::types::TemplateTypeInfo<physical_quantity, true>
	{
		PhysicalQuantityTypeInfo(): RTT::types::TemplateTypeInfo<physical_quantity, true>( "physical_quantity" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const physical_quantity& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("physical_quantity");
			targetbag.add( new Property<std::string>("physical_quantity", "", physical_quantity_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, physical_quantity& out ) const
		{
			if ( bag.getType() == std::string("physical_quantity") ) // check the type
			{
				Property<std::string>* cs = dynamic_cast<Property<std::string>*>(bag.getProperty("physical_quantity"));

				if ( !cs )
					return false;

				std::string str = cs->get();
				if(boost::equals(str, "MONITOR_POSITION"))
				{
					out = MONITOR_POSITION;
				}
				else if(boost::equals(str, "MONITOR_VELOCITY"))
				{
					out = MONITOR_VELOCITY;
				}
				else if(boost::equals(str, "MONITOR_EFFORT"))
				{
					out = MONITOR_EFFORT;
				}
				else if(boost::equals(str, "MONITOR_TIME"))
				{
					out = MONITOR_TIME;
				}
				else
				{
					log(Error) << "Enum value (physical_quantity) not recognized." << endlog();
					return false;
				}

				return true;
			}
			else
			{
				return false; // unknown type !
			}
		}
	};

	struct EventTypeTypeInfo : public RTT::types::TemplateTypeInfo<event_type, true>
	{
		EventTypeTypeInfo(): RTT::types::TemplateTypeInfo<event_type, true>( "event_type" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const event_type& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("event_type");
			targetbag.add( new Property<std::string>("event_type", "", event_type_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, event_type& out ) const
		{
			if ( bag.getType() == std::string("event_type") ) // check the type
			{
				Property<std::string>* cs = dynamic_cast<Property<std::string>*>(bag.getProperty("event_type"));

				if ( !cs )
					return false;

				std::string str = cs->get();
				if(boost::equals(str, "LEVEL"))
				{
					out = LEVEL;
				}
				else if(boost::equals(str, "EDGE"))
				{
					out = EDGE;
				}
				else
				{
					log(Error) << "Enum value (event_type) not recognized." << endlog();
					return false;
				}

				return true;
			}
			else
			{
				return false; // unknown type !
			}
		}
	};

	struct CompareTypeTypeInfo : public RTT::types::TemplateTypeInfo<compare_type, true>
	{
		CompareTypeTypeInfo(): RTT::types::TemplateTypeInfo<compare_type, true>( "compare_type" )
		{ }

		// this is a helper function, which is called by composeType() of the same class:
		virtual bool decomposeTypeImpl(const compare_type& in, RTT::PropertyBag& targetbag ) const
		{
			targetbag.setType("compare_type");
			targetbag.add( new Property<std::string>("compare_type", "", compare_type_tostring(in) ) );
			return true;
		}

		virtual bool composeTypeImpl(const RTT::PropertyBag& bag, compare_type& out ) const
		{
			if ( bag.getType() == std::string("compare_type") ) // check the type
			{
				Property<std::string>* cs = dynamic_cast<Property<std::string>*>(bag.getProperty("compare_type"));

				if ( !cs )
					return false;

				std::string str = cs->get();
				if(boost::equals(str, "LESS"))
				{
					out = LESS;
				}
				else if(boost::equals(str, "LESS_EQUAL"))
				{
					out = LESS_EQUAL;
				}
				else if(boost::equals(str, "EQUAL"))
				{
					out = EQUAL;
				}
				else if(boost::equals(str, "GREATER_EQUAL"))
				{
					out = GREATER_EQUAL;
				}
				else if(boost::equals(str, "GREATER"))
				{
					out = GREATER;
				}
				else
				{
					log(Error) << "Enum value (compare_type) not recognized." << endlog();
					return false;
				}

				return true;
			}
			else
			{
				return false; // unknown type !
			}
		}
	};

}
