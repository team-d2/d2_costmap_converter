#ifndef D2__COSTMAP_CLIPPER__COSTMAP_CLIPPER_NODE_HPP_
#define D2__COSTMAP_CLIPPER__COSTMAP_CLIPPER_NODE_HPP_


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "Eigen/Dense"

#include "d2_costmap_clipper/visibility.hpp"

namespace d2::costmap_clipper
{

class CostMapClipperNode : public rclcpp::Node
{
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
  
public:
  static constexpr auto kDefaultNodeName = "costmap_clipper";

  D2__COSTMAP_CLIPPER_PUBLIC
  inline CostMapClipperNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    clip_cell_width_(this->declare_parameter("clip_cell_width", 10)),
    clipped_costmap_publisher_(this->create_clipped_costmap_publisher()),
    pose_subscription_(this->create_pose_subscription()),
    costmap_subscription_(this->create_costmap_subscription())
  {
  }

  D2__COSTMAP_CLIPPER_PUBLIC
  explicit inline CostMapClipperNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CostMapClipperNode(node_name, "", options)
  {
  }

  D2__COSTMAP_CLIPPER_PUBLIC
  explicit inline CostMapClipperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CostMapClipperNode(kDefaultNodeName, "", options)
  {
  }



  ~CostMapClipperNode() override {}

private:
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr create_clipped_costmap_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
    };
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_publisher<OccupancyGridMsg>(
      "costmap_clipped", rclcpp::QoS(10).best_effort(), options);
  }

  rclcpp::Subscription<PoseMsg>::SharedPtr create_pose_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability
    };
    return this->create_subscription<PoseMsg>(
      "robot_pose", rclcpp::QoS(10).best_effort(),
      [this](PoseMsg::ConstSharedPtr msg){this->clip_costmap(std::move(msg));}, options);
  }

  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr create_costmap_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability
    };
    return this->create_subscription<OccupancyGridMsg>(
      "costmap", rclcpp::QoS(10).transient_local(),
      [this](OccupancyGridMsg::ConstSharedPtr msg){this->set_map_msg(std::move(msg));}, options);
  }

  void set_map_msg(OccupancyGridMsg::ConstSharedPtr costmap_msg)
  {
    if (costmap_msg->info.resolution <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Costmap resolution is invalid: %f", costmap_msg->info.resolution);
      return;
    }
    frame_id_ = costmap_msg->header.frame_id;
    map_load_time_ = costmap_msg->info.map_load_time;
    map_resolution_ = costmap_msg->info.resolution;
    map_origin_.translation() = Eigen::Vector3d(
      costmap_msg->info.origin.position.x,
      costmap_msg->info.origin.position.y,
      costmap_msg->info.origin.position.z);
    Eigen::Quaterniond q(
      costmap_msg->info.origin.orientation.w,
      costmap_msg->info.origin.orientation.x,
      costmap_msg->info.origin.orientation.y,
      costmap_msg->info.origin.orientation.z);
    map_origin_.linear() = q.toRotationMatrix();

    map_ = Eigen::Map<const Eigen::MatrixX<std::int8_t>>(
      costmap_msg->data.data(),
      costmap_msg->info.width,
      costmap_msg->info.height);
  }

  void clip_costmap(PoseMsg::ConstSharedPtr pose_msg)
  {
    if (frame_id_ == "")
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Costmap is empty");
      return;
    }

    if (frame_id_ != pose_msg->header.frame_id)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Costmap frame_id (%s) does not match pose frame_id (%s)",
        frame_id_.c_str(),
        pose_msg->header.frame_id.c_str());
      return;
    }

    Eigen::Vector3d position(
      pose_msg->pose.pose.position.x,
      pose_msg->pose.pose.position.y,
      pose_msg->pose.pose.position.z);
    Eigen::Vector3d on_map_position = map_origin_.inverse() * position;

    int costmap_clipped_centor_index_x = std::round(on_map_position.x() / map_resolution_);
    int costmap_clipped_centor_index_y = std::round(on_map_position.y() / map_resolution_);

    int costmap_clipped_min_index_x = 
      std::clamp(costmap_clipped_centor_index_x - clip_cell_width_, 0, static_cast<int>(map_.cols()));
    int costmap_clipped_min_index_y = 
      std::clamp(costmap_clipped_centor_index_y - clip_cell_width_, 0, static_cast<int>(map_.rows()));
    
    int costmap_clipped_max_index_x = 
      std::clamp(costmap_clipped_centor_index_x + clip_cell_width_, 0, static_cast<int>(map_.cols()));
    int costmap_clipped_max_index_y =
      std::clamp(costmap_clipped_centor_index_y + clip_cell_width_, 0, static_cast<int>(map_.rows()));

    auto costmap_clipped_msg = std::make_unique<OccupancyGridMsg>();
    costmap_clipped_msg->header = pose_msg->header;
    costmap_clipped_msg->info.width = costmap_clipped_max_index_x - costmap_clipped_min_index_x;
    costmap_clipped_msg->info.height = costmap_clipped_max_index_y - costmap_clipped_min_index_y;
    if (costmap_clipped_msg->info.width == 0 || costmap_clipped_msg->info.height == 0)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "pose is out of costmap range");
      return;
    }

    costmap_clipped_msg->info.map_load_time = map_load_time_;
    costmap_clipped_msg->info.resolution = map_resolution_;
    costmap_clipped_msg->info.origin.position.x = 
      map_origin_.translation().x() + costmap_clipped_min_index_x * map_resolution_;
    costmap_clipped_msg->info.origin.position.y =
      map_origin_.translation().y() + costmap_clipped_min_index_y * map_resolution_;
    costmap_clipped_msg->info.origin.position.z = map_origin_.translation().z();
    Eigen::Quaterniond q(map_origin_.rotation());
    costmap_clipped_msg->info.origin.orientation.x = q.x();
    costmap_clipped_msg->info.origin.orientation.y = q.y();
    costmap_clipped_msg->info.origin.orientation.z = q.z();
    costmap_clipped_msg->info.origin.orientation.w = q.w();

    costmap_clipped_msg->data.resize(
      costmap_clipped_msg->info.width * costmap_clipped_msg->info.height);
    
    auto clipped_map = Eigen::Map<Eigen::MatrixX<std::int8_t>>(
      costmap_clipped_msg->data.data(),
      costmap_clipped_msg->info.width,
      costmap_clipped_msg->info.height);

    clipped_map = map_.block(
      costmap_clipped_min_index_x,
      costmap_clipped_min_index_y,
      costmap_clipped_msg->info.width,
      costmap_clipped_msg->info.height);
    
    clipped_costmap_publisher_->publish(std::move(costmap_clipped_msg));
  }

  int clip_cell_width_;

  std::string frame_id_;
  rclcpp::Time map_load_time_;
  double map_resolution_;
  Eigen::Isometry3d map_origin_;
  Eigen::MatrixX<std::int8_t> map_;

  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr clipped_costmap_publisher_;

  rclcpp::Subscription<PoseMsg>::SharedPtr pose_subscription_;
  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr costmap_subscription_;
};

}

#endif  // D2__COSTMAP_CLIPPER__COSTMAP_CLIPPER_NODE_HPP_