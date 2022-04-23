#include "makeroute.hpp"

#include <array>
#include <vector>

#include "../ishihalib_cpp_gen/types/circle.hpp"
#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "visualize.hpp"

std::vector<PointTargetData> route;

struct InscribedCircle {
	InscribedCircle(char mode, ishihalib::Point center, double r) : mode_(mode), center_(center), r_(r) {
	}
	char mode_;	 //"s,z,c,d"
	ishihalib::Point center_;
	double r_;
	double begin_;
	double end_;
};
std::vector<InscribedCircle> viaCircle;
void add_line_targets(std::vector<PointTargetData> &tgs, ishihalib::LineSeg lineseg, double ds) {
	double t = 0;
	double theta = arg(lineseg.b_.get_complex() - lineseg.a_.get_complex());
	if (lineseg.length() == 0) return;
	double dt = ds / lineseg.length();
	while (t < 1) {
		PointTargetData p;
		p.pos[0] = lineseg.a_.x_ * (1. - t) + lineseg.b_.x_ * t;
		p.pos[1] = lineseg.a_.y_ * (1. - t) + lineseg.b_.y_ * t;
		p.pos[2] = theta;

		tgs.push_back(p);
		t += dt;
	}
}

void add_arc_targets(std::vector<PointTargetData> &tgs, ishihalib::Point center, double r, double theta_b, double theta_e, double ds) {
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
		tgs.push_back(p);
		t += dt;
	}
}
void add_arc_targets(std::vector<PointTargetData> &tgs, InscribedCircle c, double ds) {
	add_arc_targets(tgs, c.center_, c.r_, c.begin_, c.end_, ds);
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
	viaCircle = {
		InscribedCircle('c', ishihalib::Point(300 / 1000., 300 / 1000.), 0),
		InscribedCircle('c', ishihalib::Point(800 / 1000., 800 / 1000.), 0.3),
		InscribedCircle('s', ishihalib::Point(800 / 1000., 1600 / 1000.), 0.3),
		InscribedCircle('d', ishihalib::Point(2400 / 1000., 1600 / 1000.), 0.3),
		InscribedCircle('z', ishihalib::Point(2400 / 1000., 800 / 1000.), 0.3),
		InscribedCircle('c', ishihalib::Point(2900 / 1000., 300 / 1000.), 0),
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
	for (size_t i = 1; i < viaCircle.size() - 1; i++) {
		printf("r:%lf, b:%lf, e:%lf\n", viaCircle[i].r_, viaCircle[i].begin_, viaCircle[i].end_);
		add_arc_targets(route, viaCircle[i], ds);
	}
	return;
	ishihalib::Point a(800 / 1000., 2000 / 1000.), b(2400 / 1000., 2000 / 1000.);
	ishihalib::LineSeg ls(a, b);
	add_line_targets(route, ls, ds);
	ishihalib::Point center(800 / 1000., 1600 / 1000.);
	add_arc_targets(route, center, 400 / 1000., -M_PI_2, -3. * M_PI_2, ds);
}

void route_visualize() {
	size_t num = route.size();
	for (size_t i = 0; i < num; i += ((num + 39) / 40)) {
		ishihalib::Point p(route[i].pos[0], route[i].pos[1]);
		visualizer->rviz2.draw_point(p, "route", i);
	}
}
