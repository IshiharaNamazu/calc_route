#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "obstacleData.hpp"
#include "visualize.hpp"
ObstacleData obstacleData("./src/calc_route/myField.csv");

RouteVisualize::SharedPtr visualizer;
using namespace std::chrono_literals;
int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exec;
	auto node2 = visualizer = std::make_shared<RouteVisualize>();

	exec.add_node(node2);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}