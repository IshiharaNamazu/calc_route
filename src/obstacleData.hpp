#pragma once
#include <assert.h>

#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"

class ObstacleData {
  private:
	std::vector<std::vector<ishihalib::Point>> Obstacles;

  public:
	ObstacleData(std::string fileName = "./src/calc_route/ABU2019Field.csv") {
		std::fstream ifs(fileName.c_str());

		if (ifs.fail()) {
			std::cerr << "Failed to open Obstacle.csv" << std::endl;
			return;
		}
		std::string objStr;
		while (std::getline(ifs, objStr)) {
			std::vector<int> pointdata;	 //座標のデータ[mm]
			int tmp = 0;
			int tmp2 = 0;  //加算の計算結果
			int sig = 1;   //減算する場合
			for (auto& i : objStr) {
				if (i == ' ') continue;
				if (i == '+') {
					tmp2 += tmp * sig;
					tmp = 0;
					sig = 1;
					continue;
				}
				if (i == '-') {
					tmp2 += tmp * sig;
					tmp = 0;
					sig = -1;
					continue;
				}
				if (i == ',') {
					pointdata.push_back(tmp2 + tmp * sig);
					tmp = 0;
					tmp2 = 0;
					sig = 1;
					continue;
				}
				if (i < '0' || i > '9') {  //#,etc..
					break;
				} else {
					tmp *= 10;
					tmp += i - '0';
				}
			}

			pointdata.push_back(tmp2 + tmp * sig);
			if (pointdata.size() % 2) continue;	 //数値の個数は偶数でなくてはならない
			if (pointdata.size() == 4) {		 // x1,y1,x2,y2
				double x1 = pointdata[0] / 1000., y1 = pointdata[1] / 1000.,
					   x2 = pointdata[2] / 1000., y2 = pointdata[3] / 1000.;
				std::vector<ishihalib::Point> obstacle;
				obstacle.push_back(ishihalib::Point(x1, y1));
				obstacle.push_back(ishihalib::Point(x1, y2));
				obstacle.push_back(ishihalib::Point(x2, y2));
				obstacle.push_back(ishihalib::Point(x2, y1));
				Obstacles.push_back(obstacle);
			} else if (pointdata.size() > 4) {	// xy1,xy2,xy3...
				std::vector<ishihalib::Point> obstacle;
				for (size_t i = 0; i < pointdata.size(); i += 2) {
					obstacle.push_back(ishihalib::Point(pointdata[i] / 1000., pointdata[i + 1] / 1000.));
				}
				Obstacles.push_back(obstacle);
			}
		}
	}
	size_t size() {
		return Obstacles.size();
	}
	std::vector<ishihalib::Point> get_Object(int i) {
		return Obstacles.at(i);
	}
};

extern ObstacleData obstacleData;