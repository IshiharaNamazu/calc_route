#pragma once

#include <cstdio>
#include <random>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "obstacleData.hpp"
#include "visualize.hpp"
struct PointNode;
extern std::vector<PointNode> pointTree;

struct PointNode : public ishihalib::Point {
	int parent_;
	std::vector<int> child_;
	double cost_;
	PointNode(ishihalib::Point p) : Point(p), parent_(-1), cost_(0) {
	}
	PointNode(ishihalib::Point p, int parent, double cost) : Point(p), parent_(parent), cost_(cost) {}
	void declease_cost(double d) {
		cost_ -= d;
		for (auto &i : child_) {
			pointTree[i].declease_cost(d);
		}
	}
};

class RRTStar {
	bool crossing_field_object(ishihalib::LineSeg lineseg) {
		for (size_t i = 0; i < obstacleData.size(); i++) {
			std::vector<ishihalib::Point> obj = obstacleData.get_Object(i);
			for (size_t j = 0; j < obj.size(); j++) {
				ishihalib::LineSeg lineseg2(obj[obj.size() - 1], obj[0]);
				if (j != 0) lineseg2 = ishihalib::LineSeg(obj[j - 1], obj[j]);
				if (ishihalib::LineSeg::crossing(lineseg, lineseg2)) {
					return true;
				}
			}
		}
		return false;
	}
	bool collision_field_object(ishihalib::LineSeg lineseg, double d) {
		for (size_t i = 0; i < obstacleData.size(); i++) {
			std::vector<ishihalib::Point> obj = obstacleData.get_Object(i);
			for (size_t j = 0; j < obj.size(); j++) {
				ishihalib::LineSeg lineseg2(obj[obj.size() - 1], obj[0]);
				if (j != 0) lineseg2 = ishihalib::LineSeg(obj[j - 1], obj[j]);
				if (ishihalib::LineSeg::distance(lineseg, lineseg2) < d) {
					return true;
				}
			}
		}
		return false;
	}

  public:
	RRTStar(ishihalib::Point start = ishihalib::Point(0.5, 0.5), ishihalib::Point goal = ishihalib::Point(4.5 + 1.95 - 0.5, 8.0 + 0.5)) : start_(start), goal_(goal) {
		pointTree.push_back(start);
		std::random_device seed_gen;
		rand_engine = std::default_random_engine(seed_gen());

		ishihalib::Point fldBegin = obstacleData.get_Object(0).at(0);
		ishihalib::Point fldEnd = obstacleData.get_Object(0).at(2);
		FIELD_X_MAX = fldEnd.x_;
		FIELD_Y_MAX = fldEnd.y_;
		FIELD_X_MIN = fldBegin.x_;
		FIELD_Y_MIN = fldBegin.y_;

		randx = std::uniform_real_distribution<>(FIELD_X_MIN, FIELD_X_MAX);
		randy = std::uniform_real_distribution<>(FIELD_Y_MIN, FIELD_Y_MAX);
	}

	double visualize();

	void calc();

	double get_cost(ishihalib::Point before, ishihalib::Point after) {
		ishihalib::LineSeg ls(before, after);
		// if (collision_field_object(ls, 0.3)) return -1;
		if (crossing_field_object(ls)) return -1;
		return ls.length();
	}

	ishihalib::Point get_random_point() {
		return ishihalib::Point(randx(rand_engine), randy(rand_engine));
	}

  private:
	double FIELD_X_MAX;
	double FIELD_Y_MAX;
	double FIELD_X_MIN;
	double FIELD_Y_MIN;
	std::default_random_engine rand_engine;
	std::uniform_real_distribution<> randx;
	std::uniform_real_distribution<> randy;
	ishihalib::Point start_, goal_;
};
extern RRTStar rrtstar;
