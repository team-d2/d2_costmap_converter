#ifndef D2__COSTMAP_POLYGON_GENERATOR__COSTMAP_POLYGON_GENERATOR_NODE_HPP_
#define D2__COSTMAP_POLYGON_GENERATOR__COSTMAP_POLYGON_GENERATOR_NODE_HPP_


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp"

#include "d2_costmap_polygon_generator/visibility.hpp"

namespace d2::costmap_polygon_generator
{

class CostMapPolygonGeneratorNode : public rclcpp::Node
{
  using ObstacleArrayMsg = d2_costmap_converter_msgs::msg::ObstacleArrayMsg;
  using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
  
public:
  static constexpr auto kDefaultNodeName = "costmap_polygon_generator";

  D2__COSTMAP_POLYGON_GENERATOR_PUBLIC
  inline CostMapPolygonGeneratorNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    map_cost_threshold_(this->declare_parameter("map.obstacle.cost_threshold", 50)),
    obstacle_array_publisher_(this->obstacle_array_publisher()),
    costmap_subscription_(this->create_costmap_subscription())
  {
  }

  D2__COSTMAP_POLYGON_GENERATOR_PUBLIC
  explicit inline CostMapPolygonGeneratorNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CostMapPolygonGeneratorNode(node_name, "", options)
  {
  }

  D2__COSTMAP_POLYGON_GENERATOR_PUBLIC
  explicit inline CostMapPolygonGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CostMapPolygonGeneratorNode(kDefaultNodeName, "", options)
  {
  }



  ~CostMapPolygonGeneratorNode() override {}

private:
  static inline bool is_line(const cv::Point2i& p1, const cv::Point2i& p2, const cv::Point2i& p3)
  {
    double area = 
      p1.x * (p2.y - p3.y) +
      p2.x * (p3.y - p1.y) +
      p3.x * (p1.y - p2.y);
    return area == 0;
  }

  rclcpp::Publisher<ObstacleArrayMsg>::SharedPtr obstacle_array_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options =
    {
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability,
    };
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_publisher<ObstacleArrayMsg>(
      "obstacle_array", rclcpp::QoS(10).best_effort(), options);
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
      "costmap", rclcpp::QoS(10).best_effort(),
      [this](OccupancyGridMsg::ConstSharedPtr msg){this->generate_obstacle_array(std::move(msg));}, options);
  }

  void generate_obstacle_array(OccupancyGridMsg::ConstSharedPtr costmap_msg)
  {
    Eigen::MatrixX<std::int8_t> costmap_transposed =
      Eigen::Map<const Eigen::MatrixX<std::int8_t>>(
        costmap_msg->data.data(),
        costmap_msg->info.width,
        costmap_msg->info.height);

    // OpenCVのMatに変換（符号なし8bitにキャストする）
    cv::Mat img(costmap_transposed.cols(), costmap_transposed.rows(), CV_8SC1, costmap_transposed.data());
    cv::Mat img_u8;
    img.convertTo(img_u8, CV_8UC1);

    cv::Mat masked_img;
    cv::inRange(img_u8, cv::Scalar(map_cost_threshold_), cv::Scalar(255), masked_img);

    // --- 小さいノイズを除去（モルフォロジー開処理）---
    cv::Mat mask_clean_img;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 3x3のカーネル
    cv::morphologyEx(masked_img, mask_clean_img, cv::MORPH_CLOSE, kernel);

    // 輪郭検出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_clean_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    auto obstacle_array_msg = std::make_unique<ObstacleArrayMsg>();
    obstacle_array_msg->header = costmap_msg->header;

    Eigen::Isometry3d map_origin;
    map_origin.translation() = Eigen::Vector3d(
      costmap_msg->info.origin.position.x,
      costmap_msg->info.origin.position.y,
      costmap_msg->info.origin.position.z);
    Eigen::Quaterniond q(
      costmap_msg->info.origin.orientation.w,
      costmap_msg->info.origin.orientation.x,
      costmap_msg->info.origin.orientation.y,
      costmap_msg->info.origin.orientation.z);
    map_origin.linear() = q.toRotationMatrix();

    const auto to_point32_msg_data = [&](cv::Point2i point)
    {
      Eigen::Vector3d relative_point(
        point.x * costmap_msg->info.resolution,
        point.y * costmap_msg->info.resolution, 0.0);
      Eigen::Vector3d point_eigen = map_origin * relative_point;
      geometry_msgs::msg::Point32 point_msg_data;
      point_msg_data.x = point_eigen.x();
      point_msg_data.y = point_eigen.y();
      point_msg_data.z = point_eigen.z();
      return point_msg_data;
    };

    std::size_t obstacle_id = 0;
    for (const auto& contour : contours) {
      d2_costmap_converter_msgs::msg::ObstacleMsg obstacle_msg_data;
      obstacle_msg_data.header = costmap_msg->header;
      obstacle_msg_data.id = obstacle_id;
      if (contour.size() < 3) {
        for (const auto& piccel : contour) {
          const auto point_msg_data = to_point32_msg_data(piccel);
          obstacle_msg_data.polygon.points.push_back(point_msg_data);
        }
      }
      else {
        if (!is_line(contour.back(), contour[0], contour[1])) {
          const auto point_msg_data = to_point32_msg_data(contour[0]);
          obstacle_msg_data.polygon.points.push_back(point_msg_data);
        }
        for (size_t i = 1; i < contour.size() - 1; ++i) {
          if (!is_line(contour[i - 1], contour[i], contour[i + 1])) {
            const auto point_msg_data = to_point32_msg_data(contour[i]);
            obstacle_msg_data.polygon.points.push_back(point_msg_data);
          }
        }
        if (!is_line(contour[contour.size() - 2], contour.back(), contour[0])) {
          const auto point_msg_data = to_point32_msg_data(contour.back());
          obstacle_msg_data.polygon.points.push_back(point_msg_data);
        }
      }
      obstacle_array_msg->obstacles.push_back(obstacle_msg_data);
      ++obstacle_id;
    }

    obstacle_array_publisher_->publish(std::move(obstacle_array_msg));

    // 元画像をカラー化（赤線を描くため）
    cv::Mat img_color;
    cv::cvtColor(mask_clean_img, img_color, cv::COLOR_GRAY2BGR);

    // 赤線で輪郭を描画
    cv::drawContours(img_color, contours, -1, cv::Scalar(0, 0, 255), 1);  // BGR=(0,0,255)

    // 結果表示
    cv::imshow("Original with Contours", img_color);
    cv::waitKey(1);

    // // 結果表示
    // for (size_t i = 0; i < contours.size(); ++i) {
    //     std::cout << "Region " << i << ":\n";
    //     for (const auto& pt : contours[i]) {
    //         std::cout << "  (" << pt.x << ", " << pt.y << ")\n";
    //     }
    // }
  }

  std::int_fast8_t map_cost_threshold_;

  rclcpp::Publisher<ObstacleArrayMsg>::SharedPtr obstacle_array_publisher_;

  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr costmap_subscription_;
};

}

#endif  // D2__COSTMAP_POLYGON_GENERATOR__COSTMAP_POLYGON_GENERATOR_NODE_HPP_