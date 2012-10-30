#include "ExecutiveHelpers.hpp"

#include <tf/tf.h>
#include <cassert>
#include <rtt/TaskContext.hpp>

namespace YouBot
{
	using namespace std;

	void homogeneous_to_xyzypr(const vector<double>& H, vector<double>& xyzypr)
	{
		assert(H.size() == 16 && xyzypr.size() == 6);

		xyzypr[0] = H[3];
		xyzypr[1] = H[7];
		xyzypr[2] = H[11];

		btMatrix3x3 rotMatrix(H[0], H[1], H[2], H[4], H[5], H[6], H[8], H[9], H[10]);

		btScalar y, p, r;
		rotMatrix.getEulerYPR(y, p, r);
		xyzypr[3] = y;
		xyzypr[4] = p;
		xyzypr[5] = r;
	}

	std::string& make_event(std::string& s, const std::string& event)
	{
		using namespace RTT;
		char tmpstr[max_event_length];
		if(s.capacity() < max_event_length)
		  log(Error) << "make_event: event string capacity < max_event_length." << endlog();

		snprintf(tmpstr, max_event_length, "%s", event.c_str());
		s.insert(0, tmpstr, max_event_length);
		return s;
	}

}
