#pragma once

#include <IRobotGripperService.hpp>
#include <puma260.hpp>

namespace Puma260
{

  using namespace RTT;
  using namespace RTT::types;  
  using namespace std;

  class Puma260GripperService: public Hilas::IRobotGripperService
  {

    public:

      Puma260GripperService(const string& name, TaskContext* parent, long clientID);
      ~Puma260GripperService();

      void displayGripperStatus();

    private:

      bool calibrate();
      void cleanup();
      void stop();
      void checkForErrors();
      void setGripperSetpoints();
  };

}
