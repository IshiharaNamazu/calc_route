#include <array>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"

struct PointTargetData {
	std::array<double, 3> pos;
	std::array<double, 3> velocity;
	std::array<double, 3> accel;
};

std::vector<PointTargetData> route;

void add_line_targets(std::vector<PointTargetData> &tgs, ishihalib::LineSeg lineseg, double ds) {
	double t = 0;
	double theta = arg(lineseg.b_.get_complex() - lineseg.a_.get_complex());
	if (lineseg.length() == 0) return;
	double dt = ds / lineseg.length();
	while (t < 1) {
		PointTargetData p;
		p.pos[0] = lineseg.a_.x_ * (1 - t) + lineseg.b_.x_ * t;
		p.pos[1] = lineseg.a_.y_ * (1 - t) + lineseg.b_.y_ * t;
		p.pos[2] = theta;

		tgs.push_back(p);
		t += dt;
	}
}

void add_arc_targets(std::vector<PointTargetData> &tgs, ishihalib::Point center, double r, double theta_b, double theta_e, double ds) {
	double length = abs(theta_e - theta_b) * r;
	double dt = ds / length;
	double t = 0;
	while (t < 1) {
		PointTargetData p;
		double theta = theta_b * (1 - t) + theta_e * t;
		p.pos[0] = center.x_ + r * cos(theta);
		p.pos[1] = center.y_ + r * sin(theta);
		p.pos[2] = (theta_e - theta_b > 0) ? theta + M_PI_2 : theta - M_PI_2;
		tgs.push_back(p);
		t += dt;
	}
}

void makeroute() {
	ishihalib::Point a(800, 2000), b(2400, 2000);
	ishihalib::LineSeg ls(a, b);
	add_line_targets(route, ls, 10 / 1000.);
}