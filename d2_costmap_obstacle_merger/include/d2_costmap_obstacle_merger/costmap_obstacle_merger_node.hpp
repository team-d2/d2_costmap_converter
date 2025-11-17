#ifndef D2__COSTMAP_OBSTACLE_MERGER__COSTMAP_OBSTACLE_MERGER_NODE_HPP_
#define D2__COSTMAP_OBSTACLE_MERGER__COSTMAP_OBSTACLE_MERGER_NODE_HPP_


#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "Eigen/Dense"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "d2_costmap_obstacle_merger/visibility.hpp"

namespace d2::costmap_obstacle_merger
{

class CostMapObstacleMergerNode : public rclcpp::Node
{
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
  
public:
  static constexpr auto kDefaultNodeName = "costmap_obstacle_merger";

  D2__OBSTACLE_MERGER_PUBLIC
  inline CostMapObstacleMergerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    lidar_point_radius_(this->declare_parameter("lidar.point.radius", 0.3)),
    obstacle_cost_(this->declare_parameter("map.obstacle.cost", 100)),
    frame_id_(this->declare_parameter("frame_id", "map")),
    costmap_obstacle_merged_publisher_(this->create_clipped_costmap_publisher()),
    lidar_points_subscription_(this->create_pose_subscription()),
    costmap_subscription_(this->create_costmap_subscription())
  {
  }

  D2__OBSTACLE_MERGER_PUBLIC
  explicit inline CostMapObstacleMergerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CostMapObstacleMergerNode(node_name, "", options)
  {
  }

  D2__OBSTACLE_MERGER_PUBLIC
  explicit inline CostMapObstacleMergerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CostMapObstacleMergerNode(kDefaultNodeName, "", options)
  {
  }

  ~CostMapObstacleMergerNode() override {}

private:
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr create_clipped_costmap_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability,
    };
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_publisher<OccupancyGridMsg>(
      "costmap_with_obstacle", rclcpp::QoS(10).best_effort(), options);
  }

  rclcpp::Subscription<PointCloudMsg>::SharedPtr create_pose_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability,
    };
    return this->create_subscription<PointCloudMsg>(
      "lidar/points", rclcpp::QoS(10).best_effort(),
      [this](PointCloudMsg::ConstSharedPtr msg){this->update_lidar_points(std::move(msg));}, options);
  }

  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr create_costmap_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability,
    };
    return this->create_subscription<OccupancyGridMsg>(
      "costmap", rclcpp::QoS(10).best_effort(),
      [this](OccupancyGridMsg::ConstSharedPtr msg){this->update_costmap(std::move(msg));}, options);
  }

  void update_costmap(OccupancyGridMsg::ConstSharedPtr costmap_msg)
  {
    if (costmap_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "costmap frame_id '%s' does not match frame_id '%s'. "
        "Topic is ignored.",
        costmap_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    costmap_msg_ = std::move(costmap_msg);

    this->merge(costmap_msg_->header.stamp);
  }

  void update_lidar_points(PointCloudMsg::ConstSharedPtr lidar_points_msg)
  {
    if (lidar_points_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "lidar points frame_id '%s' does not match frame_id '%s'. "
        "Topic is ignored.",
        lidar_points_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
    pcl::fromROSMsg(*lidar_points_msg, pcl_point_cloud);
    lidar_points_.clear();
    for (const auto& point : pcl_point_cloud.points) {
      lidar_points_.emplace_back(point.x, point.y, point.z);
    }

    if (costmap_msg_ == nullptr) {
      return;
    }

    this->merge(lidar_points_msg->header.stamp);
  }

  void merge(const builtin_interfaces::msg::Time& stamp)
  {
    auto merged_costmap_msg = std::make_unique<OccupancyGridMsg>(*costmap_msg_);
    merged_costmap_msg->data.resize(
      merged_costmap_msg->info.width * merged_costmap_msg->info.height);
    merged_costmap_msg->header.frame_id = frame_id_;
    merged_costmap_msg->header.stamp = stamp;
    merged_costmap_msg->info = costmap_msg_->info;

    auto merged_map = Eigen::Map<Eigen::MatrixX<std::int8_t>>(
      merged_costmap_msg->data.data(),
      merged_costmap_msg->info.width,
      merged_costmap_msg->info.height);

    const double lidar_point_cell_radius = lidar_point_radius_ / merged_costmap_msg->info.resolution;

    Eigen::Isometry3d map_origin;
    map_origin.translation() = Eigen::Vector3d(
      merged_costmap_msg->info.origin.position.x,
      merged_costmap_msg->info.origin.position.y,
      merged_costmap_msg->info.origin.position.z);
    Eigen::Quaterniond q(
      merged_costmap_msg->info.origin.orientation.w,
      merged_costmap_msg->info.origin.orientation.x,
      merged_costmap_msg->info.origin.orientation.y,
      merged_costmap_msg->info.origin.orientation.z);
    map_origin.linear() = q.toRotationMatrix();

    Eigen::Isometry3d map_origin_inv = map_origin.inverse();

    for (const auto& point : lidar_points_) {
      const auto relative_point = map_origin_inv * point;
      const double cell_y_centor = relative_point.y() / merged_costmap_msg->info.resolution;
      const int cell_y_min = std::max(0.0, std::floor(cell_y_centor - lidar_point_cell_radius));
      const int cell_y_max = std::min(merged_costmap_msg->info.height - 1.0, std::ceil(cell_y_centor + lidar_point_cell_radius));
      for (int cell_y = cell_y_min; cell_y <= cell_y_max; ++cell_y) {
        const double cell_x_centor = relative_point.x() / merged_costmap_msg->info.resolution;
        const double cdll_y_delta = cell_y - cell_y_centor;
        const double cell_x_delta = std::sqrt(std::max(0.0, lidar_point_cell_radius * lidar_point_cell_radius - cdll_y_delta * cdll_y_delta));
        const int cell_x_min = std::max(0.0, std::floor(cell_x_centor - cell_x_delta));
        const int cell_x_max = std::min(merged_costmap_msg->info.width - 1.0, std::ceil(cell_x_centor + cell_x_delta));
        const int cell_x_range = cell_x_max - cell_x_min + 1;
        if (cell_x_range <= 0) {
          continue;
        }
        auto segment = merged_map.col(cell_y).segment(cell_x_min, cell_x_range);
        segment = segment.unaryExpr([this](std::int8_t value) -> std::int8_t
        {
          if (value == -1) {
            return 100;
          }
          return std::min(value + obstacle_cost_, 100);
        });
      }
    }

    costmap_obstacle_merged_publisher_->publish(std::move(merged_costmap_msg));
  }

  double lidar_point_radius_;
  int obstacle_cost_;
  std::string frame_id_;

  OccupancyGridMsg::ConstSharedPtr costmap_msg_;
  std::vector<Eigen::Vector3d> lidar_points_;

  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr costmap_obstacle_merged_publisher_;

  rclcpp::Subscription<PointCloudMsg>::SharedPtr lidar_points_subscription_;
  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr costmap_subscription_;
};

}

#endif  // D2__COSTMAP_OBSTACLE_MERGER__COSTMAP_OBSTACLE_MERGER_NODE_HPP_