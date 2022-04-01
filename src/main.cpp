#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "RRTStar.hpp"
#include "obstacleData.hpp"
#include "visualize.hpp"
ObstacleData obstacleData;
RRTStar rrtstar;
std::vector<PointNode> pointTree;
RouteVisualize::SharedPtr visualizer;
using namespace std::chrono_literals;
class calcRoute : public rclcpp::Node {
  public:
	calcRoute() : Node("calc_route") {
		timer_ = this->create_wall_timer(1s, std::bind(&calcRoute::timer_callback, this));
	}

  private:
	void timer_callback() {
		// RCLCPP_INFO(this->get_logger(), "%d\n", obstacleData.size());
		//  rrtstar.calc();
	}
	rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exec;
	auto node1 = std::make_shared<calcRoute>();
	auto node2 = visualizer = std::make_shared<RouteVisualize>();
	exec.add_node(node1);
	exec.add_node(node2);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}