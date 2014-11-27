#include <interfaces/IRobotKinematics.hpp>

#include <puma260.hpp>

namespace Puma260
{

using namespace KDL;
using namespace RTT;
using namespace boost;

class Puma260Kinematics : public Hilas::IRobotKinematics
{

public:

    Puma260Kinematics(std::string const& name);
    ~Puma260Kinematics();

private:

    void createKinematicChain();
    void forwardKinematic();
    void differentialKinematic();
};

}