
#include "RRTStar.hpp"

#include <cstdio>
#include <random>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "obstacleData.hpp"
#include "visualize.hpp"

void RRTStar::calc() {
	ishihalib::Point p = get_random_point();
	// visualizer->draw_point(pointTree[0], "start");
	// visualizer->draw_point(p);
	int nearPoint = -1;
	double mincost = -1;
	for (size_t i = 0; i < pointTree.size(); i++) {
		double cost = get_cost(pointTree[i], p);
		if (cost > 0) {
			cost += pointTree[i].cost_;
			if (mincost < 0) {
				nearPoint = i;
				mincost = cost;
			} else if (mincost > cost) {
				nearPoint = i;
				mincost = cost;
			}
		}
	}
	if (mincost > 0) {	//ノード追加
		PointNode child(p);
		child.cost_ = mincost;
		child.parent_ = nearPoint;
		pointTree[nearPoint].child_.push_back(pointTree.size());
		pointTree.push_back(child);

		if (true) {
			for (size_t i = 0; i < (pointTree.size() - 1); i++) {
				if (pointTree[i].cost_ < pointTree.back().cost_) {
					double addcost = get_cost(pointTree.back(), pointTree[i]);
					if (addcost > 0) {
						double cost_dif = pointTree[i].cost_ - (pointTree.back().cost_ + addcost);
						if (cost_dif > 0) {
							printf("here***********************************************************\n");
							pointTree[i].parent_ = pointTree.size() - 1;  //最後のノードを親に
							pointTree.back().child_.push_back(i);
							pointTree[i].declease_cost(cost_dif);
						}
					}
				}
			}
		}
	}
}

double RRTStar::visualize() {
	int nearPoint = -1;
	double mincost = -1;
	for (size_t i = 0; i < pointTree.size(); i++) {
		double cost = get_cost(pointTree[i], goal_);

		if (cost > 0) {
			cost += pointTree[i].cost_;
			if (mincost < 0) {
				nearPoint = i;
				mincost = cost;
			} else if (mincost > cost) {
				nearPoint = i;
				mincost = cost;
			}
		}
	}
	std::vector<ishihalib::LineSeg> lines;
	if (mincost > 0) {
		lines.push_back(ishihalib::LineSeg(goal_, pointTree[nearPoint]));
		while (nearPoint != 0) {
			int per = pointTree[nearPoint].parent_;

			lines.push_back(ishihalib::LineSeg(pointTree[nearPoint], pointTree[per]));
			nearPoint = per;
		}
	}
	// lines.clear();
	visualizer->draw_line_segs(lines);
	visualizer->draw_point(pointTree[nearPoint], "start");
	visualizer->draw_point(start_, "start");
	visualizer->draw_point(goal_, "goal");
	return mincost;
}
extern RRTStar rrtstar;