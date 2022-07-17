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

void route_ramp(double ds);
struct InscribedCircle {
	InscribedCircle(char mode, ishihalib::Point center, double r, double v = MAX_VELOCITY) : mode_(mode), center_(center), r_(r), v_(v) {
	}
	char mode_;	 //"s,z,c,d"
	ishihalib::Point center_;
	double r_;
	double begin_;
	double end_;
	double v_;
};
std::vector<InscribedCircle> viaCircle;
void add_line_targets(std::vector<PointTargetData> &tgs, ishihalib::LineSeg lineseg, double ds, double v) {
	double t = 0;
	double theta = arg(lineseg.b_.get_complex() - lineseg.a_.get_complex());
	if (lineseg.length() == 0) return;
	double dt = ds / lineseg.length();
	while (t < 1) {
		PointTargetData p;
		p.pos[0] = lineseg.a_.x_ * (1. - t) + lineseg.b_.x_ * t;
		p.pos[1] = lineseg.a_.y_ * (1. - t) + lineseg.b_.y_ * t;
		p.pos[2] = theta;
		if (t == 0.0)
			p.set_polarvelocity(v, theta);
		else
			p.set_polarvelocity(MAX_VELOCITY, theta);

		tgs.push_back(p);
		t += dt;
	}
}

void add_arc_targets(std::vector<PointTargetData> &tgs, ishihalib::Point center, double r, double theta_b, double theta_e, double ds, double v) {
	double length = (abs(theta_e - theta_b)) * r;
	if (length == 0) return;
	double dt = ds / length;
	double t = 0;
	while (t < 1) {
		PointTargetData p;
		double theta = theta_b * (1 - t) + theta_e * t;
		p.pos[0] = center.x_ + r * cos(theta);
		p.pos[1] = center.y_ + r * sin(theta);
		p.pos[2] = (theta_e - theta_b > 0) ? theta + M_PI_2 : theta - M_PI_2;
		p.set_polarvelocity(v, p.pos[2], 1. / r);
		tgs.push_back(p);
		t += dt;
	}
}
void add_arc_targets(std::vector<PointTargetData> &tgs, InscribedCircle c, double ds) {
	add_arc_targets(tgs, c.center_, c.r_, c.begin_, c.end_, ds, c.v_);
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
	constexpr double MACHINE_SIZE_X = 700.0;
	constexpr double MACHINE_SIZE_Y = 700.0;
	constexpr double R = 445.0 / 1000;
	viaCircle = {
		// InscribedCircle('c', ishihalib::Point((MACHINE_SIZE_X/2) / 1000., (MACHINE_SIZE_Y/2) / 1000.), 0, 0),
		// InscribedCircle('c', ishihalib::Point((4500-700) / 1000., (500+MACHINE_SIZE_Y/2) / 1000.), 0, 0),
		// InscribedCircle('c', ishihalib::Point((3500) / 1000., (1281+19) / 1000.), R),
		InscribedCircle('c', ishihalib::Point((MACHINE_SIZE_X / 2) / 1000., (1281 + 38 + MACHINE_SIZE_Y / 2) / 1000.), 0, 0),
		InscribedCircle('d', ishihalib::Point((1200) / 1000., (4100 - 940.5) / 1000.), R),
		InscribedCircle('z', ishihalib::Point((2200) / 1000., (4100 - 940.5) / 1000.), R),
		InscribedCircle('s', ishihalib::Point((3200) / 1000., (4100 - 940.5) / 1000.), R),
		InscribedCircle('d', ishihalib::Point((4500 - 500 - MACHINE_SIZE_X / 2) / 1000., (4100 - 940.5) / 1000.), 0, 0),
	};
	for (size_t i = 1; i < viaCircle.size(); i++) {
		connect_circle(viaCircle[i - 1], viaCircle[i]);
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
		add_line_targets(route, ishihalib::LineSeg(a, b), ds, viaCircle[i - 1].v_);
		if (i != viaCircle.size() - 1) add_arc_targets(route, viaCircle[i], ds);
	}
	route[0].set_polarvelocity(0.1, route[0].get_velocity_arg());
	route[route.size() - 1].set_polarvelocity(0.1, route[route.size() - 1].get_velocity_arg());
	route_ramp(ds);
	return;
}

void route_visualize() {
	std::ofstream outputfile("./src/calc_route/route.txt");
	size_t num = route.size();
	size_t drawnum = 200;
	for (size_t i = 0; i < num; i += ((num + drawnum - 1) / drawnum)) {
		double t = route[i].get_velocity_size() / MAX_VELOCITY;
		ishihalib::Point p(route[i].pos[0], route[i].pos[1]);
		ishihalib::LineSeg v(p, p.get_complex() + std::complex<double>(route[i].velocity[0], route[i].velocity[1]));
		visualizer->rviz2.draw_point(p, "route", i, 1, 1 - t, 1 - t);
		visualizer->rviz2.draw_line_seg(v, "velocity", i);
	}
	outputfile << "n = " << num << "\n";
	outputfile << "\n\n\n{\n";
	for (size_t i = 0; i < num; i++) {
		outputfile << " std::array<MoveTarget, 3>{\n";
		outputfile << "  MoveTarget(" << route[i].pos[1] << ',' << route[i].pos[0] << ',' << 0 << "),\n";
		outputfile << "  MoveTarget(" << route[i].velocity[1] << ',' << route[i].velocity[0] << ',' << 0 << "),\n";
		outputfile << "  MoveTarget(" << 0 << ',' << 0 << ',' << 0 << "),\n";
		outputfile << "},";
	}
	outputfile << "}";
	outputfile.close();
}

void route_ramp(double ds) {
	// v=at x=1/2at^2 2ax=v^2 2adx=2vdv dv/dx=a/v
	for (size_t i = 1; i < route.size(); i++) {
		double dvdx = MAX_ACCEL / route[i].get_velocity_size();

		double v = std::min(MAX_VELOCITY, (route[i - 1].get_velocity_size() + dvdx * ds));
		if (v < route[i].get_velocity_size()) {
			route[i].set_polarvelocity(v, route[i].get_velocity_arg());
		}
	}
	for (size_t i = route.size() - 2; i != 0; i--) {
		double dvdx = MAX_ACCEL / route[i].get_velocity_size();

		double v = std::min(MAX_VELOCITY, (route[i + 1].get_velocity_size() + dvdx * ds));
		if (v < route[i].get_velocity_size()) {
			route[i].set_polarvelocity(v, route[i].get_velocity_arg());
		}
	}
}