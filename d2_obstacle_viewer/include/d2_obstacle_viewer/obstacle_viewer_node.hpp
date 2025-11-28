#ifndef D2__OBSTACLE_VIEWER__OBSTACLE_VIEWER_NODE_HPP_
#define D2__OBSTACLE_VIEWER__OBSTACLE_VIEWER_NODE_HPP_

#include <cstdint>
#include <set>

#include "visualization_msgs/msg/marker_array.hpp"
#include "d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp"
#include "rclcpp/node.hpp"
#include "Eigen/Dense"

#include "d2_obstacle_viewer/visibility.hpp"

namespace d2::obstacle_viewer
{

class ObstacleViewerNode : public rclcpp::Node
{
  using MarkerMsg = visualization_msgs::msg::Marker;
  using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
  using ObstacleMsg = d2_costmap_converter_msgs::msg::ObstacleMsg;
  using ObstacleArrayMsg = d2_costmap_converter_msgs::msg::ObstacleArrayMsg;
  
public:
  static constexpr auto kDefaultNodeName = "obstacle_viewer";

  D2__OBSTACLE_VIEWER_PUBLIC
  inline ObstacleViewerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    marker_ns_(this->declare_parameter("marker.ns", "obstacle_marker")),
    line_width_(this->declare_parameter("marker.line_width", 0.1)),
    color_r_(this->declare_parameter("marker.color.r", 1.0)),
    color_g_(this->declare_parameter("marker.color.g", 0.0)),
    color_b_(this->declare_parameter("marker.color.b", 0.0)),
    color_a_(this->declare_parameter("marker.color.a", 1.0)),
    obstacle_array_marker_publisher_(this->create_clipped_costmap_publisher()),
    pose_subscription_(this->create_pose_subscription())
  {
  }

  D2__OBSTACLE_VIEWER_PUBLIC
  explicit inline ObstacleViewerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstacleViewerNode(node_name, "", options)
  {
  }

  D2__OBSTACLE_VIEWER_PUBLIC
  explicit inline ObstacleViewerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstacleViewerNode(kDefaultNodeName, "", options)
  {
  }



  ~ObstacleViewerNode() override {}

private:
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr create_clipped_costmap_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability,
    };
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_publisher<MarkerArrayMsg>(
      "obstacle_array/marker", rclcpp::QoS(10).best_effort(), options);
  }

  rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr create_pose_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability
    };
    return this->create_subscription<ObstacleArrayMsg>(
      "obstacle_array", rclcpp::QoS(10).best_effort(),
      [this](ObstacleArrayMsg::ConstSharedPtr msg){this->convert_to_marker_array(std::move(msg));}, options);
  }

  void convert_to_marker_array(ObstacleArrayMsg::ConstSharedPtr obstacle_array_msg)
  {
    auto marker_array_msg = std::make_unique<MarkerArrayMsg>();
    marker_array_msg->markers.reserve(obstacle_array_msg->obstacles.size());

    std::unordered_set<std::uint32_t> current_marker_ids;
    for (const auto &obstacle_msg_data : obstacle_array_msg->obstacles) {
      current_marker_ids.insert(obstacle_msg_data.id);
      marker_array_msg->markers.emplace_back(to_marker_msg_data(obstacle_msg_data));
    }
    for (const auto & last_id : last_marker_ids_) {
      if (current_marker_ids.find(last_id) != current_marker_ids.end()) {
        continue;
      }
      marker_array_msg->markers.emplace_back(to_marker_delete_msg_data(last_id));
    }

    obstacle_array_marker_publisher_->publish(std::move(marker_array_msg));
    last_marker_ids_.merge(current_marker_ids);
  }

  MarkerMsg to_marker_msg_data(const ObstacleMsg & obstacle_msg_data) const
  {
    MarkerMsg marker_msg_data;
    marker_msg_data.header = obstacle_msg_data.header;
    marker_msg_data.ns = marker_ns_;
    marker_msg_data.id = obstacle_msg_data.id;
    marker_msg_data.type = MarkerMsg::LINE_STRIP;
    marker_msg_data.action = MarkerMsg::ADD;
    marker_msg_data.scale.x = line_width_;
    marker_msg_data.color.r = color_r_;
    marker_msg_data.color.g = color_g_;
    marker_msg_data.color.b = color_b_;
    marker_msg_data.color.a = color_a_;
    marker_msg_data.frame_locked = false;
    for (const auto & point : obstacle_msg_data.polygon.points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      marker_msg_data.points.push_back(p);
    }
    if (obstacle_msg_data.polygon.points.size() >= 3) {
      marker_msg_data.points.push_back(marker_msg_data.points.front());
    }
    return marker_msg_data;
  }

  MarkerMsg to_marker_delete_msg_data(std::uint32_t id)
  {
    MarkerMsg delete_marker_msg;
    delete_marker_msg.action = MarkerMsg::DELETE;
    delete_marker_msg.ns = marker_ns_;
    delete_marker_msg.id = id;
    return delete_marker_msg;
  };

  std::string marker_ns_;
  double line_width_;
  double color_r_, color_g_, color_b_, color_a_;
  std::unordered_set<std::uint32_t> last_marker_ids_;

  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr obstacle_array_marker_publisher_;
  rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr pose_subscription_;
};

}

#endif  // D2__OBSTACLE_VIEWER__OBSTACLE_VIEWER_NODE_HPP_