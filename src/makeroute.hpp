#pragma once
#include <array>
#include <complex>
#include <vector>
struct PointTargetData {
	std::array<double, 3> pos = {0., 0., 0.};
	std::array<double, 3> velocity = {0., 0., 0.};
	std::array<double, 3> accel = {0., 0., 0.};
	void set_polarvelocity(double v, double theta, double curvature = 0) {
		velocity[0] = v * cos(theta);
		velocity[1] = v * sin(theta);
		velocity[2] = curvature * v;
	}
	double get_velocity_size() {
		return sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
	}
	double get_velocity_arg() {
		return atan2(velocity[1], velocity[0]);
	}
};

extern std::vector<PointTargetData> route;

void make_route();
void route_visualize();