#pragma once
#include <cstdio>
#include <random>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "obstacleData.hpp"

struct PointNode : public ishihalib::Point {
	int parent_;
	std::vector<int> child_;
	double cost;
};

class RRTStar {
  public:
	RRTStar() {
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

	void visualize() {
	}

	void calc() {
		ishihalib::Point p = get_random_point();
		printf("%lf, %lf\n", p.x_, p.y_);
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
};

extern RRTStar rrtstar;