#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "obstacleData.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
using namespace std::chrono_literals;
class RouteVisualize : public rclcpp::Node {
  public:
	RouteVisualize()
		: Node("RouteVisualize") {
		marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("vizmarker", 10);
		timer_ = this->create_wall_timer(1000ms, std::bind(&RouteVisualize::timer_callback, this));
	}

  private:
	void timer_callback() {
		line_list.header.frame_id = "/map";
		line_list.header.stamp = this->get_clock()->now();
		line_list.ns = "obstacles";
		line_list.action = visualization_msgs::msg::Marker::ADD;
		line_list.pose.orientation.w = 1.0;
		line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

		line_list.id = 0;
		line_list.scale.x = 0.01;
		line_list.scale.y = 0.01;
		line_list.color.r = 1.0;
		line_list.color.a = 1.0;

		for (size_t i = 0; i < obstacleData.size(); i++) {
			std::vector<ishihalib::Point> obj = obstacleData.get_Object(i);
			geometry_msgs::msg::Point p;

			p.x = obj[0].x_;  //オブジェクト描画
			p.y = obj[0].y_;
			p.z = 0;
			line_list.points.push_back(p);
			for (size_t j = 1; j < obj.size(); j++) {
				p.x = obj[j].x_;
				p.y = obj[j].y_;
				p.z = 0;
				line_list.points.push_back(p);
				line_list.points.push_back(p);
			}
			p.x = obj[0].x_;
			p.y = obj[0].y_;
			p.z = 0;
			line_list.points.push_back(p);
		}

		marker_pub->publish(line_list);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
	visualization_msgs::msg::Marker line_list;
};
