#include <functional>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "makeroute.hpp"
#include "obstacleData.hpp"
#include "visualize.hpp"
ObstacleData obstacleData("./src/calc_route/myField.csv");

RouteVisualize::SharedPtr visualizer;
using namespace std::chrono_literals;
using namespace std;
int main(int argc, char* argv[]) {
	make_route();
	cout << route.size() << endl;
	rclcpp::init(argc, argv);
	visualizer = std::make_shared<RouteVisualize>();
	rclcpp::spin(visualizer);
	rclcpp::shutdown();
	return 0;
}