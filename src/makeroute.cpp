#include "makeroute.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <vector>

#include "../ishihalib_cpp_gen/types/circle.hpp"
#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "param.h"
#include "visualize.hpp"

std::vector<PointTargetData> route;
void write_pyroute();
void write_cpp_arrays();
void write_move_targets();

void route_ramp(double ds);
struct InscribedCircle {
	InscribedCircle(char mode, ishihalib::Point center, double r, double v = MAX_VELOCITY, int8_t along_turn_mode = 0, double first_angle = -100) : mode_(mode), center_(center), r_(r), v_(v), first_angle_(first_angle), along_turn_mode_(along_turn_mode) {
	}
	char mode_;	 //"s,z,c,d"
	ishihalib::Point center_;
	double r_;
	double begin_;
	double end_;
	double v_;
	double first_angle_;
	int8_t along_turn_mode_;  //円弧に沿って旋回するか 0:無効 1:回す 2:begin+first_angleで入り、回す
};
std::vector<InscribedCircle> viaCircle;
void add_line_targets(std::vector<PointTargetData> &tgs, ishihalib::LineSeg lineseg, double ds, double v, double last_angle = -100) {
	double first_angle = M_PI_2;  /////////////////////////////////////////こいつが初期角度決めてしまう
	if (tgs.size() != 0) first_angle = tgs.back().pos[2];
	double t = 0;
	double theta = first_angle;
	if (lineseg.length() == 0) return;
	double ex = (lineseg.b_.x_ - lineseg.a_.x_) / lineseg.length();
	double ey = (lineseg.b_.y_ - lineseg.a_.y_) / lineseg.length();
	double dt = ds / lineseg.length();
	while (t < 1) {
		PointTargetData p;
		p.pos[0] = lineseg.a_.x_ * (1. - t) + lineseg.b_.x_ * t;
		p.pos[1] = lineseg.a_.y_ * (1. - t) + lineseg.b_.y_ * t;
		if (last_angle < -50) {
			theta = first_angle;
		}

		else {
			theta = first_angle * (1. - t) + last_angle * t;
		}
		p.pos[2] = theta;
		if (t == 0.0) {
			p.velocity[0] = ex * v;
			p.velocity[1] = ey * v;
			if (last_angle < -50)
				p.velocity[2] = ((last_angle - first_angle) / lineseg.length()) * v;
			else
				p.velocity[2] = 0;
		} else {
			p.velocity[0] = ex * MAX_VELOCITY;
			p.velocity[1] = ey * MAX_VELOCITY;
			if (last_angle < -50)
				p.velocity[2] = ((last_angle - first_angle) / lineseg.length()) * MAX_VELOCITY;
			else
				p.velocity[2] = 0;
		}

		tgs.push_back(p);
		t += dt;
	}
}

void add_arc_targets(std::vector<PointTargetData> &tgs, ishihalib::Point center, double r, double theta_b, double theta_e, double ds, double v, int8_t along_turn_mode) {
	double first_angle = tgs.back().pos[2];
	double length = (abs(theta_e - theta_b)) * r;
	if (length == 0) return;
	double dt = ds / length;
	double t = 0;
	while (t < 1) {
		PointTargetData p;
		double theta = theta_b * (1 - t) + theta_e * t;
		p.pos[0] = center.x_ + r * cos(theta);
		p.pos[1] = center.y_ + r * sin(theta);
		if (along_turn_mode == 1 || along_turn_mode == 2)
			p.pos[2] = (first_angle + theta - theta_b);
		else
			p.pos[2] = first_angle;
		double velarg = (theta_e - theta_b > 0) ? theta + M_PI_2 : theta - M_PI_2;
		p.velocity[0] = cos(velarg) * v;
		p.velocity[1] = sin(velarg) * v;
		if (along_turn_mode == 1 || along_turn_mode == 2)
			p.velocity[2] = v / r;
		else
			p.velocity[2] = 0;
		tgs.push_back(p);
		t += dt;
	}
}
void add_arc_targets(std::vector<PointTargetData> &tgs, InscribedCircle c, double ds) {
	add_arc_targets(tgs, c.center_, c.r_, c.begin_, c.end_, ds, c.v_, c.along_turn_mode_);
}
void connect_circle(InscribedCircle &c1, InscribedCircle &c2) {
	ishihalib::Circle theta0(2 * M_PI), theta1(2 * M_PI), ret(2 * M_PI);
	theta0 = arg(c2.center_.get_complex() - c1.center_.get_complex());
	double L = ishihalib::Point::distance(c2.center_, c1.center_);
	if (c2.mode_ == 'c' || c2.mode_ == 'd') {
		theta1 = asin((c1.r_ - c2.r_) / L);
		if (c2.mode_ == 'd') {
			ret = theta0 + M_PI_2 - theta1;
		}
		if (c2.mode_ == 'c') {
			ret = theta0 - M_PI_2 + theta1;
		}
		c1.end_ = ret;
		c2.begin_ = ret;
	}
	if (c2.mode_ == 's' || c2.mode_ == 'z') {
		theta1 = atan2((c1.r_ + c2.r_), L);
		if (c2.mode_ == 's') {
			ret = theta0 + theta1;
			c1.end_ = ret - M_PI_2;
			c2.begin_ = ret + M_PI_2;
		}
		if (c2.mode_ == 'z') {
			ret = theta0 - theta1;
			c1.end_ = ret + M_PI_2;
			c2.begin_ = ret - M_PI_2;
		}
	}
}

void make_route() {
	double ds = 10 / 1000.;
	constexpr double MACHINE_SIZE_X = 560.0;
	constexpr double MACHINE_SIZE_Y = 560.0;
	constexpr double R = (499.) / 1000.;
	constexpr double margin = 50. / 1000;

	viaCircle = {
		InscribedCircle('c', ishihalib::Point((MACHINE_SIZE_X / 2) / 1000. + margin, (MACHINE_SIZE_Y / 2) / 1000. + margin), 0, 0),
		InscribedCircle('c', ishihalib::Point((4500 - 700) / 1000., (MACHINE_SIZE_Y / 2) / 1000. + margin), 0, 0),
		InscribedCircle('c', ishihalib::Point((3500) / 1000., (1281 + 19) / 1000.), 0.45),
		InscribedCircle('c', ishihalib::Point((MACHINE_SIZE_X / 2) / 1000. + margin, (1281 + 38 + MACHINE_SIZE_Y / 2) / 1000. + margin), 0, 0),
		InscribedCircle('d', ishihalib::Point((1200) / 1000., (4100 - 940.5) / 1000.), R, MAX_VELOCITY, 2, 0),
		InscribedCircle('z', ishihalib::Point((2200) / 1000., (4100 - 940.5) / 1000.), R, MAX_VELOCITY, 2),
		InscribedCircle('s', ishihalib::Point((3200) / 1000., (4100 - 940.5) / 1000.), R, MAX_VELOCITY, 2),
		InscribedCircle('d', ishihalib::Point((4500 - MACHINE_SIZE_Y / 2) / 1000. - 0.2, (4100 - 940.5) / 1000.), 0, 0, 0, M_PI_2),
	};
	for (size_t i = 1; i < viaCircle.size(); i++) {
		connect_circle(viaCircle[i - 1], viaCircle[i]);
		if (viaCircle[i].along_turn_mode_ == 2) {
			viaCircle[i].first_angle_ += viaCircle[i].begin_;
		}
	}
	for (size_t i = 1; i < viaCircle.size() - 1; i++) {
		if (viaCircle[i].mode_ == 'c' || viaCircle[i].mode_ == 'z') {
			while (viaCircle[i].begin_ > viaCircle[i].end_) {
				viaCircle[i].end_ += M_PI * 2;
			}
		}
		if (viaCircle[i].mode_ == 'd' || viaCircle[i].mode_ == 's') {
			while (viaCircle[i].begin_ < viaCircle[i].end_) {
				viaCircle[i].end_ -= M_PI * 2;
			}
		}
	}
	for (size_t i = 1; i < viaCircle.size(); i++) {
		printf("r:%lf, b:%lf, e:%lf\n", viaCircle[i].r_, viaCircle[i].begin_, viaCircle[i].end_);
		ishihalib::Point a, b;
		a = ishihalib::Point(viaCircle[i - 1].center_.get_complex() + std::polar(viaCircle[i - 1].r_, viaCircle[i - 1].end_));
		b = ishihalib::Point(viaCircle[i].center_.get_complex() + std::polar(viaCircle[i].r_, viaCircle[i].begin_));
		add_line_targets(route, ishihalib::LineSeg(a, b), ds, viaCircle[i - 1].v_, viaCircle[i].first_angle_);
		if (i != viaCircle.size() - 1) add_arc_targets(route, viaCircle[i], ds);
	}
	route[0].set_polarvelocity(0, route[0].get_velocity_arg());
	route[route.size() - 1].set_polarvelocity(0, route[route.size() - 1].get_velocity_arg());
	route_ramp(ds);
	write_pyroute();
	write_cpp_arrays();
	write_move_targets();
	return;
}

void route_visualize() {
	size_t num = route.size();
	size_t drawnum = 200;
	for (size_t i = 0; i < num; i += ((num + drawnum - 1) / drawnum)) {
		double t = route[i].get_velocity_size() / MAX_VELOCITY;
		ishihalib::Point p(route[i].pos[0], route[i].pos[1]);
		ishihalib::LineSeg v(p, p.get_complex() + std::complex<double>(route[i].velocity[0], route[i].velocity[1]));
		visualizer->rviz2.draw_point(p, "route", i, 1, 1 - t, 1 - t);
		visualizer->rviz2.draw_line_seg(v, "velocity", i);
	}
}

void route_ramp(double ds) {
	// v=at x=1/2at^2 2ax=v^2 2adx=2vdv dv/dx=a/v
	for (size_t i = 1; i < route.size(); i++) {
		route[i].accel = std::array<double, 3>{0, 0, 0};
		if (0 == route[i].velocity[0] * route[i].velocity[0] + route[i].velocity[1] * route[i].velocity[1]) continue;
		double vv = route[i - 1].velocity[0] * route[i - 1].velocity[0] + route[i - 1].velocity[1] * route[i - 1].velocity[1];
		double dv = -sqrt(vv) + sqrt(vv + 2. * MAX_ACCEL * ds);

		double v = std::min(MAX_VELOCITY, (route[i - 1].get_velocity_size() + dv));
		if (v < route[i].get_velocity_size()) {
			double k = v / route[i].get_velocity_size();
			route[i].velocity[0] *= k;
			route[i].velocity[1] *= k;
			route[i].velocity[2] *= k;
		}
	}
	for (int i = (int)route.size() - 2; i != -1; i--) {
		route[i].accel = std::array<double, 3>{0, 0, 0};
		if (0 == route[i].velocity[0] * route[i].velocity[0] + route[i].velocity[1] * route[i].velocity[1]) continue;
		double vv = route[i + 1].velocity[0] * route[i + 1].velocity[0] + route[i + 1].velocity[1] * route[i + 1].velocity[1];
		double dv = -sqrt(vv) + sqrt(vv + 2. * MAX_ACCEL * ds);

		double v = std::min(MAX_VELOCITY, (route[i + 1].get_velocity_size() + dv));
		if (v < route[i].get_velocity_size()) {
			double k = v / route[i].get_velocity_size();
			route[i].velocity[0] *= k;
			route[i].velocity[1] *= k;
			route[i].velocity[2] *= k;
		}
	}
}

void write_pyroute() {
	//製作時のガバによりxy軸が反転している
	std::ofstream outputfile("./src/calc_route/src/product/movie/pyroute.py");
	size_t num = route.size();
	// size_t drawnum = 200;
	double t = 0;
	outputfile << "route = [\n";
	for (size_t i = 0; i < num; i++) {
		double vv = route[i].velocity[0] * route[i].velocity[0] + route[i].velocity[1] * route[i].velocity[1];
		if (i != 0 && vv >= 1e-12) {  //速度が小さすぎるものは無視
			outputfile << "    [ " << t << " , " << route[i].pos[1] << " , " << route[i].pos[0] << " , " << (M_PI_2 - route[i].pos[2]) << " ],\n";
			double dx = route[i].pos[1] - route[i - 1].pos[1];
			double dy = route[i].pos[0] - route[i - 1].pos[0];
			t += sqrt((dx * dx + dy * dy) / vv);
		}
	}
	outputfile << "]";
	outputfile.close();
}

void write_cpp_arrays() {
	//製作時のガバによりxy軸が反転している
	std::ofstream outputfile("./src/calc_route/src/product/array.cpp");
	size_t num = route.size();
	// size_t drawnum = 200;
	outputfile << "#include <vector>\n#include <array>";
	outputfile << "std::vector<std::array<std::array<double, 3>>> route = {n";
	for (size_t i = 0; i < num; i++) {
		double vv = route[i].velocity[0] * route[i].velocity[0] + route[i].velocity[1] * route[i].velocity[1];
		if (vv >= 1e-12) {	//速度が小さすぎるものは無視
			outputfile << "  std::array<std::array<double, 3>, 3>{\n";
			outputfile << "    std::array<double, 3>{" << route[i].pos[1] << " , " << route[i].pos[0] << " , " << (M_PI_2 - route[i].pos[2]) << " },\n";
			outputfile << "    std::array<double, 3>{" << route[i].velocity[1] << " , " << route[i].velocity[0] << " , " << (-route[i].velocity[2]) << " },\n";
			outputfile << "    std::array<double, 3>{" << route[i].accel[1] << " , " << route[i].accel[0] << " , " << (-route[i].accel[2]) << " },\n";
			outputfile << "  },\n";
		}
	}
	outputfile << "};\n";
	outputfile.close();
}
void write_move_targets() {
	//製作時のガバによりxy軸が反転している
	std::ofstream outputfile("./src/calc_route/src/product/move_targets.cpp");
	size_t num = route.size();
	outputfile << "n = " << num << "\n";
	outputfile << "\n\n\nstd::vector<std::array<MoveTarget, 3>> route = {\n";
	for (size_t i = 0; i < num; i++) {
		outputfile << " std::array<MoveTarget, 3>{\n";
		outputfile << "  MoveTarget(" << route[i].pos[1] << ',' << route[i].pos[0] << ',' << (M_PI_2 - route[i].pos[2]) << "),\n";
		outputfile << "  MoveTarget(" << route[i].velocity[1] << ',' << route[i].velocity[0] << ',' << -route[i].velocity[2] << "),\n";
		outputfile << "  MoveTarget(" << route[i].accel[1] << ',' << route[i].accel[0] << ',' << -route[i].accel[2] << "),\n";
		outputfile << "},";
	}
	outputfile << "}";
	outputfile.close();
}