#pragma once

#include <IRobotGripperService.hpp>
#include <wam7dof.hpp>

namespace Wam7dof
{

  using namespace RTT;
  using namespace RTT::types;  
  using namespace std;

  class Wam7dofGripperService: public Hilas::IRobotGripperService
  {

    public:

      Wam7dofGripperService(const string& name, TaskContext* parent, long i_clientID);
      ~Wam7dofGripperService();

      void displayGripperStatus();

    private:

      bool calibrate();
      void cleanup();
      void stop();
      void checkForErrors();
      void setGripperSetpoints();
  };

}
