
#include "visualize.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "../ishihalib_cpp_gen/utility/geometry.hpp"
#include "../ishihalib_ros2/rviz2/rvizdraw.hpp"
#include "obstacleData.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
using namespace std::chrono_literals;
// extern RRTStar rrtstar;
void RouteVisualize::field_viewer() {
	std::vector<ishihalib::LineSeg> lines;
	for (size_t i = 0; i < obstacleData.size(); i++) {
		std::vector<ishihalib::Point> obj = obstacleData.get_Object(i);
		for (size_t j = 1; j < obj.size() + 1; j++) {
			lines.push_back(ishihalib::LineSeg(obj[j - 1], obj[j % (obj.size())]));
		}
	}
	rviz2.draw_line_segs(lines, "field", 0, 1, 0, 0);
	return;
}