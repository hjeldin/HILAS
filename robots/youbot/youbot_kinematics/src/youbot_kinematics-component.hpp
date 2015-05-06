#include <interfaces/IRobotKinematics.hpp>

#include "generic/Units.hpp"
#include <youbot/ProtocolDefinitions.hpp>

#include "youbot/YouBotBase.hpp"
#include "generic/ConfigFile.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"

#include <youBot.hpp>

namespace YouBot
{

using namespace KDL;
using namespace RTT;
using namespace boost;

class YouBotKinematics : public Hilas::IRobotKinematics
{

public:

    YouBotKinematics(std::string const& name);
    ~YouBotKinematics();

private:

    void twistToJointVelocities(quantity<si::velocity> l,quantity<si::velocity> t, quantity<si::angular_velocity> a);
    void createKinematicChain();
    void assignJointToChain();
    void forwardKinematic();
    void differentialKinematic();

    youbot::FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;
    boost::scoped_ptr<youbot::ConfigFile> configfile;
    youbot::FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;    
};

}