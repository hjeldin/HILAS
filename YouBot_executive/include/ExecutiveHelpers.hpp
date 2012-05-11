#pragma once

#include <vector>
#include "ExecutiveTypes.hpp"

namespace YouBot
{
	using namespace std;

	void homogeneous_to_xyzypr(const vector<double>& H, vector<double>& xyzypr);

	void Multiply(const vector<double>& lhs, const vector<double>& r, vector<double>& output);
	void MultiplyH(const vector<double>& lhs, const vector<double>& rhs, vector<double>& output);
	void Sum(const vector<double>& lhs, const vector<double>& rhs, vector<double>& output);

	std::string& make_event(std::string& s, const std::string& event);
}
