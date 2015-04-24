#include <interfaces/IRobotKinematics.hpp>

#include <wam7dof.hpp>

namespace Wam7dof
{

using namespace KDL;
using namespace RTT;
using namespace boost;

class Wam7dofKinematics : public Hilas::IRobotKinematics
{

public:

    Wam7dofKinematics(std::string const& name);
    ~Wam7dofKinematics();

private:

    void createKinematicChain();
    void forwardKinematic();
    void differentialKinematic();
};

}