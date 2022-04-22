#pragma once
#ifndef VISUALIZE_HPP_
#define VISUALIZE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
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
	}

  public:
	RouteVisualize()
		: Node("RouteVisualize") {
		marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("vizmarker", 10);
		timer_ = this->create_wall_timer(1s, std::bind(&RouteVisualize::timer_callback, this));
	}

  private:
  public:
	void draw_point(ishihalib::Point P, std::string _ns = "point", int _id = 0) {
		visualization_msgs::msg::Marker point;
		point.header.frame_id = "/map";
		point.header.stamp = this->get_clock()->now();
		point.ns = _ns;
		point.id = _id;
		point.type = visualization_msgs::msg::Marker::POINTS;

		point.scale.x = 0.05;
		point.scale.y = 0.05;

		geometry_msgs::msg::Point p;
		p.x = P.x_;
		p.y = P.y_;
		point.points.push_back(p);

		point.color.g = 1.0f;
		point.color.a = 1.0;
		marker_pub->publish(point);
	}
	void draw_line_seg(ishihalib::LineSeg lineseg, std::string _ns = "lineseg", int _id = 0) {
		visualization_msgs::msg::Marker line;
		line.header.frame_id = "/map";
		line.header.stamp = this->get_clock()->now();
		line.ns = _ns;
		line.action = visualization_msgs::msg::Marker::ADD;
		line.pose.orientation.w = 1.0;
		line.type = visualization_msgs::msg::Marker::LINE_LIST;

		line.id = _id;
		line.scale.x = 0.01;
		line.scale.y = 0.01;
		line.color.r = 1.0;
		line.color.a = 1.0;

		geometry_msgs::msg::Point p;
		p.x = lineseg.a_.x_;
		p.y = lineseg.a_.y_;
		line.points.push_back(p);

		p.x = lineseg.b_.x_;
		p.y = lineseg.b_.y_;
		line.points.push_back(p);

		marker_pub->publish(line);
	}
	void draw_line_segs(std::vector<ishihalib::LineSeg> linesegs, std::string _ns = "linesegs", int _id = 0, float r = 0, float g = 1, float b = 0) {
		visualization_msgs::msg::Marker lines;
		lines.header.frame_id = "/map";
		lines.header.stamp = this->get_clock()->now();
		lines.ns = _ns;
		lines.action = visualization_msgs::msg::Marker::ADD;
		lines.pose.orientation.w = 1.0;
		lines.type = visualization_msgs::msg::Marker::LINE_LIST;

		lines.id = _id;
		lines.scale.x = 0.01;
		lines.scale.y = 0.01;
		lines.color.r = r;
		lines.color.g = g;
		lines.color.b = b;
		lines.color.a = 1.0;
		for (auto &lineseg : linesegs) {
			geometry_msgs::msg::Point p;
			p.x = lineseg.a_.x_;
			p.y = lineseg.a_.y_;
			lines.points.push_back(p);

			p.x = lineseg.b_.x_;
			p.y = lineseg.b_.y_;
			lines.points.push_back(p);
		}

		marker_pub->publish(lines);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
	visualization_msgs::msg::Marker line_list;
	using SharedPtr = std::shared_ptr<RouteVisualize>;
};

extern RouteVisualize::SharedPtr visualizer;
#endif
