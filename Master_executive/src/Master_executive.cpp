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
 * Yury Brodskiy 
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

#include "Master_executive.hpp"

#include <boost/thread/thread.hpp>
#include <math.h>

using namespace RTT;
using namespace std;

namespace YouBot
{

static const unsigned int SIZE_CART_STIFFNESS=9;
static const double BASIC_CART_STIFFNESS[]={40,40,40,0,0,0,0,0,0};

Master_executive::Master_executive(const string& name) : TaskContext(name)
{
	this->addPort("EnergyQuanta",EnergyQuanta).doc("Port to supply system with energy");
	this->addPort("EnergyState1",EnergyState1).doc("Connect component energy state");

	this->addEventPort("stiffness_slider", stiffness_slider).doc("Slider to go from pure driving to arm+base control state. Expects input values between -1 and 1.");
	this->addPort("CartSpaceStiffness", CartSpaceStiffness).doc("");

	this->addPort("submit_quota", submit_quota).doc("");

	this->addOperation("submitEnergyQuanta", &Master_executive::submitEnergyQuanta, this, OwnThread);
	this->addOperation("getEnergyState1", &Master_executive::getEnergyState1, this, OwnThread);

	this->addOperation("setCartesianStiffness", &Master_executive::setCartesianStiffness, this, OwnThread);

	// Debugging/introspection properties
	this->addProperty("CartSpaceStiffness", m_CartSpaceStiffness.data);
	this->addProperty("quanta", m_quota);

	m_EnergyQuanta.data.resize(2, 0.0);
	m_EnergyState1.data.resize(1, 0.0);
	EnergyQuanta.setDataSample(m_EnergyQuanta);

	m_quota = 100;
	m_submit_quota.data = false;
	m_submit_quota_prev.data = false;

	m_CartSpaceStiffness.data.resize(SIZE_CART_STIFFNESS, 0.0);
	m_CartSpaceStiffness_orig.data.resize(SIZE_CART_STIFFNESS, 0.0);
	m_CartSpaceStiffness_orig.data.assign(BASIC_CART_STIFFNESS, BASIC_CART_STIFFNESS+SIZE_CART_STIFFNESS);

	CartSpaceStiffness.setDataSample(m_CartSpaceStiffness);

	m_stiffness_slider.data.resize(1, -1); // 0 percent
}

Master_executive::~Master_executive()
{
}

bool Master_executive::startHook ()
{
  setCartSpaceStiffness();
  return true;
}

void Master_executive::updateHook()
{
  if(stiffness_slider.read(m_stiffness_slider) == NewData)
  { 
    setCartSpaceStiffness();
  }

  if(submit_quota.read(m_submit_quota) == NewData)
  {
    if(m_submit_quota.data != m_submit_quota_prev.data && m_submit_quota.data == true)
    {
      submitEnergyQuanta(m_quota);
    }

    m_submit_quota_prev.data = m_submit_quota.data;
  }
}

void Master_executive::stopHook()
{
	m_CartSpaceStiffness.data.assign(SIZE_CART_STIFFNESS ,0.0);
	CartSpaceStiffness.write(m_CartSpaceStiffness);
	
	submitEnergyQuanta(-abs(getEnergyState1()));
	TaskContext::stopHook();
}

void Master_executive::submitEnergyQuanta(double joules)
{
	m_EnergyQuanta.data[0]= joules;
	m_EnergyQuanta.data[1]= (m_EnergyQuanta.data[1] +1);
	EnergyQuanta.write(m_EnergyQuanta);
}

double Master_executive::getEnergyState1()
{
  EnergyState1.read(m_EnergyState1);
  return m_EnergyState1.data[0];
}

void Master_executive::setCartesianStiffness(vector<double> stiffness_c)
{  
  if(stiffness_c.size() != SIZE_CART_STIFFNESS)
  {
    log(Error) << "setCartesianStiffness - expects a " << SIZE_CART_STIFFNESS << " dimensional vector" << endlog();
    return;
  }

  m_CartSpaceStiffness_orig.data.assign(stiffness_c.begin(),stiffness_c.end());
  setCartSpaceStiffness();
}

void Master_executive::setCartSpaceStiffness()
{  
  double percentage = (m_stiffness_slider.data[0] + 1) / 2; // For the Logitech joystick the input will be between -1 and +1
  if(percentage >= 0.0 && percentage <= 1.0)
  {
    for(unsigned int i = 0; i < SIZE_CART_STIFFNESS; ++i)
    {
      m_CartSpaceStiffness.data[i] = m_CartSpaceStiffness_orig.data[i] * percentage;
    }
  }
  CartSpaceStiffness.write(m_CartSpaceStiffness); // no 'commit' necessary)
}

}

ORO_CREATE_COMPONENT(YouBot::Master_executive)


