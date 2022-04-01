#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "obstacleData.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
using namespace std::chrono_literals;
class RouteVisualize : public rclcpp::Node {
  public:
	RouteVisualize()
		: Node("RouteVisualize") {
		marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("vizmarker", 10);
		timer_ = this->create_wall_timer(200ms, std::bind(&RouteVisualize::timer_callback, this));
	}

  private:
	void timer_callback() {
		field_viewer();
		ishihalib::Point p = rrtstar.get_random_point();
		ishihalib::LineSeg ls(rrtstar.get_random_point(), ishihalib::Point(0.1, 0.1));
		draw_line_seg(ls);
	}
	void field_viewer() {
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
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
	visualization_msgs::msg::Marker line_list;
};
