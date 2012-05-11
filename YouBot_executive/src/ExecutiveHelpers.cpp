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

	void Multiply(const vector<double>& lhs, const vector<double>& rhs, vector<double>& output)
	{
		using namespace std;

		if(lhs.size() != 16 || rhs.size() != 4 || output.size() != 4)
		{
			 throw std::out_of_range("Multiply::vector::_M_range_check");
		}

		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				output[i]+=lhs[i*4+j]*rhs[j];
			}
		}
	}
	void MultiplyH(const vector<double>& lhs, const vector<double>& rhs, vector<double>& output)
	{
		using namespace std;

		if(lhs.size() != 16 || rhs.size() != 16 || output.size() != 16)
		{
			 throw std::out_of_range("Multiply::vector::_M_range_check");
		}

		for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j<4; j++)
			{
				output[i*4+j]=0;
				for(int k=0; k<4; k++)
				{
				output[i*4+j]+=lhs[i*4+k]*rhs[k*4+j];
				}
			}
		}
	}

	void Sum(const vector<double>& lhs,const vector<double>& rhs, vector<double>& output)
	{
		using namespace std;

		if(lhs.size() != rhs.size() || output.size() != rhs.size())
		{
			 throw std::out_of_range("Sum::vector::_M_range_check");
		}

		for(unsigned int i = 0; i < rhs.size();i++)
		{
			output[i]=rhs[i]*lhs[i];
		}
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
