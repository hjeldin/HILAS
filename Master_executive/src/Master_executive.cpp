#include "Master_executive.hpp"

#include <boost/thread/thread.hpp>
#include <math.h>

using namespace Orocos;
using namespace RTT;
using namespace std;

namespace YouBot
{

static const unsigned int SIZE_CART_STIFFNESS=9;
static const double BASIC_CART_STIFFNESS[]={50,50,50,0,0,0,0,0,0};

Master_executive::Master_executive(const string& name) : TaskContext(name)
{
	this->addPort("EnergyQuanta",EnergyQuanta).doc("Port to supply system with energy");
	this->addPort("EnergyState1",EnergyState1).doc("Connect component energy state");

	this->addEventPort("stiffness_slider", stiffness_slider).doc("Slider to go from pure driving to arm+base control state. Expects input values between -1 and 1.");
	this->addPort("CartSpaceStiffness", CartSpaceStiffness).doc("");

	this->addOperation("submitEnergyQuanta", &Master_executive::submitEnergyQuanta, this, OwnThread);
	this->addOperation("getEnergyState1", &Master_executive::getEnergyState1, this, OwnThread);

	this->addOperation("setCartesianStiffness", &Master_executive::setCartesianStiffness, this, OwnThread);

	m_EnergyQuanta.data.resize(2, 0.0);
	m_EnergyState1.data.resize(1, 0.0);
	EnergyQuanta.setDataSample(m_EnergyQuanta);

	m_CartSpaceStiffness.data.resize(SIZE_CART_STIFFNESS, 0.0);
	m_CartSpaceStiffness_orig.data.resize(SIZE_CART_STIFFNESS, 0.0);
	m_CartSpaceStiffness_orig.data.assign(BASIC_CART_STIFFNESS, BASIC_CART_STIFFNESS+SIZE_CART_STIFFNESS);

	CartSpaceStiffness.setDataSample(m_CartSpaceStiffness);

	m_stiffness_slider.data.resize(1, 1); // 100 percent
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


