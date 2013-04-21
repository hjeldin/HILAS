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
 * Yury Brodskiy, Robert Wilterdink
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

#include "Adapter20Sim.h"

#include <boost/algorithm/string.hpp>

namespace common20sim {

	std::string replaceIllegalCharacter(std::string str)
	{
		str = replaceIllegalCharacter(str, "\\", "_");
		str = replaceIllegalCharacter(str, "[", "__");
		str = replaceIllegalCharacter(str, "]", "__");
		str = replaceIllegalCharacter(str, ".", "_");
		str = replaceIllegalCharacter(str, ",", "_");
		return str;
	}

	std::string replaceIllegalCharacter(std::string str, std::string pattern, std::string replacement)
	{
	  size_t found;
	  found=str.find_first_of(pattern);
    while (found != std::string::npos)
    {
      str.replace(found,1, replacement);
      found=str.find_first_of(pattern,found+replacement.length());
    }
    return str;
	}

	std::string makeShortName(std::string str)
	{
		size_t pos;
		pos=str.find_last_of("\\");
		if (pos != std::string::npos)
		{
			std::string temp;
			temp=str.substr(pos+1,temp.length()-pos-1);
			return temp;
		}
		else
		{
			return str;
		}
	}

}


