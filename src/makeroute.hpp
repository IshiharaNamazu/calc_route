#pragma once
#include <array>
#include <vector>
struct PointTargetData {
	std::array<double, 3> pos;
	std::array<double, 3> velocity;
	std::array<double, 3> accel;
};

extern std::vector<PointTargetData> route;

void make_route();
void route_visualize();