#pragma once
#include "Macho.hpp"
#include "string"
#include "vector"
namespace YouBot{
class YouBot_executive;
}
namespace FlowControl
{
	TOPSTATE(Top) {
		STATE(Top)
			//state machine interface
				//state machine wide event with defined transition
		 void e_fullControl();
		 void e_gravityMode();
		 void e_jointControl();
		 void e_catesianControl();
		 void e_guardedMove();
		  //overloaded function that does tracking
		virtual void run(YouBot::YouBot_executive* executive) {}
		virtual std::string toString();
	private:
		virtual void init();
	};

	SUBSTATE(fullControl,Top) {
		STATE(fullControl)
	public:
			void run(YouBot::YouBot_executive* executive);
			 std::string toString();
	};
	SUBSTATE(gravityMode,Top) {
		STATE(gravityMode)
	public:
			void run(YouBot::YouBot_executive* executive);
			 std::string toString();
	};
	SUBSTATE(jointControl,Top) {
		STATE(jointControl)
	public:
			void run(YouBot::YouBot_executive* executive);
			 std::string toString();
	};
	SUBSTATE(catesianControl,Top) {
		STATE(catesianControl)
	public:
			void run(YouBot::YouBot_executive* executive);
			 std::string toString();
	};
	SUBSTATE(guardedMove,Top) {
		STATE(guardedMove)

	public:
			std::vector<double> error;
			void init();
			void run(YouBot::YouBot_executive* executive);
			 std::string toString();
	};
	SUBSTATE(retractGripper,Top) {
		STATE(retractGripper)
	private:
				static const double STIFFNESS_C[2];
				static double GRIPPER_SIZE[3];
				vector<double> setPoint;
				vector<double> vecStiffness;
				vector<double> vecGripperSize;
				bool firstRun;
	public:

			void init();
			void run(YouBot::YouBot_executive* executive);
		    std::string toString();
	};

}//flowControl
void Multiply(const vector<double>& H,const vector<double>& r, vector<double>& output);
