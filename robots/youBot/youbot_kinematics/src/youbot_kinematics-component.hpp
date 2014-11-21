#include <interfaces/IRobotKinematics.hpp>

#include <generic/Units.hpp>
#include <youbot/ProtocolDefinitions.hpp>

#include "youbot/YouBotBase.hpp"
#include "generic/ConfigFile.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"

namespace YouBot
{

using namespace KDL;
using namespace RTT;
using namespace boost;

class YouBot_kinematics : public Hilas::IRobotKinematics
{

public:

    YouBot_kinematics(std::string const& name);
    ~YouBot_kinematics();

private:

    void twistToJointVelocities(quantity<si::velocity> l,quantity<si::velocity> t, quantity<si::angular_velocity> a);
    void modifyDefaultChain();
    void assignJointToChain();
    void forwardKinematic();
    void differentialKinematic();

    youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;
    boost::scoped_ptr<youbot::ConfigFile> configfile;
    youbot::FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;    
};

}