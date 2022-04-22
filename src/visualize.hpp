#pragma once
#ifndef VISUALIZE_HPP_
#define VISUALIZE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "../ishihalib_ros2/rviz2/rvizdraw.hpp"
#include "makeroute.hpp"
#include "obstacleData.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
using namespace std::chrono_literals;
class RouteVisualize : public rclcpp::Node {
	void field_viewer();
	void timer_callback() {
		field_viewer();
		route_visualize();
		rviz2.publish();
	}

  public:
	RouteVisualize()
		: Node("RouteVisualize"), rviz2(this) {
		timer_ = this->create_wall_timer(1s, std::bind(&RouteVisualize::timer_callback, this));
	}

  private:
  public:
	rclcpp::TimerBase::SharedPtr timer_;
	using SharedPtr = std::shared_ptr<RouteVisualize>;
	ishihalib::RvizDraw rviz2;
};

extern RouteVisualize::SharedPtr visualizer;
#endif
