#pragma once

#include <vector>
#include <YouBotTypes.hpp>
#include "ExecutiveTypes.hpp"

namespace YouBot
{
	using namespace std;

	void homogeneous_to_xyzypr(const vector<double>& H, vector<double>& xyzypr);

	std::string& make_event(std::string& s, const std::string& event);
}
