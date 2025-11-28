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
  using PolygonMsg = geometry_msgs::msg::Polygon;
  using Point32Msg = geometry_msgs::msg::Point32;
  
public:
  static constexpr auto kDefaultNodeName = "costmap_polygon_generator";

  D2__COSTMAP_POLYGON_GENERATOR_PUBLIC
  inline CostMapPolygonGeneratorNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    map_cost_threshold_(this->declare_parameter("map.obstacle.cost_threshold", 50)),
    expansion_allowance_(this->declare_parameter("map.obstacle.expansion_allowance", 0.5)),
    shrinkage_allowance_(this->declare_parameter("map.obstacle.shrinkage_allowance", 0.0)),
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
  std::vector<Eigen::Vector2d> contour_to_polygon(const std::vector<cv::Point2i>& contours)
  {
    std::vector<Eigen::Vector2d> polygon;
    if (contours.size() < 3) {
      for (const auto& pt : contours) {
        polygon.emplace_back(pt.x, pt.y);
      }
      return polygon;
    }

    polygon.reserve(contours.size());
    polygon.emplace_back(contours.front().x, contours.front().y);
    auto contour_itr = std::next(contours.begin());
    std::vector<cv::Point2i> middle_contours;
    auto is_line = [&middle_contours, this](const cv::Point2i& p1, const cv::Point2i& p2)
    {
      const auto p_diff_x = p2.x - p1.x;
      const auto p_diff_y = p2.y - p1.y;
      const auto p_difff_norm = std::sqrt(p_diff_x * p_diff_x + p_diff_y * p_diff_y);
      const auto det_min = -this->expansion_allowance_ * p_difff_norm;
      const auto det_max = +this->shrinkage_allowance_ * p_difff_norm;
      for (const auto& mid_pt : middle_contours) {
        const auto det = p_diff_x * (mid_pt.y - p1.y) - p_diff_y * (mid_pt.x - p1.x);
        if (det < det_min || det_max < det) {
          return false;
        }
      }
      return true;
    };
    auto last_point = contours.front();
    for (; contour_itr != contours.end(); ++contour_itr) {
      if (!is_line(last_point, *contour_itr)) {
        last_point = middle_contours.back();
        polygon.emplace_back(last_point.x, last_point.y);
        middle_contours.clear();
      }
      middle_contours.push_back(*contour_itr);
    }
    if (!is_line(last_point, contours.front())) {
      polygon.emplace_back(contours.back().x, contours.back().y);
    }
    polygon.emplace_back(contours.front().x, contours.front().y);
    return polygon;
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

    std::size_t obstacle_id = 0;
    for (const auto& contour : contours) {
      d2_costmap_converter_msgs::msg::ObstacleMsg obstacle_msg_data;
      obstacle_msg_data.header = costmap_msg->header;
      obstacle_msg_data.id = obstacle_id;
      const auto polygon = contour_to_polygon(contour);
      obstacle_msg_data.polygon.points.reserve(polygon.size());
      for (const auto & point : polygon) {
        Eigen::Vector3d map_point = map_origin * Eigen::Vector3d(
          point.x() * costmap_msg->info.resolution,
          point.y() * costmap_msg->info.resolution,
          0.0);
        Point32Msg point_msg;
        point_msg.x = static_cast<float>(map_point.x());
        point_msg.y = static_cast<float>(map_point.y());
        point_msg.z = static_cast<float>(map_point.z());
        obstacle_msg_data.polygon.points.push_back(point_msg);
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
  double expansion_allowance_, shrinkage_allowance_;

  rclcpp::Publisher<ObstacleArrayMsg>::SharedPtr obstacle_array_publisher_;

  rclcpp::Subscription<OccupancyGridMsg>::SharedPtr costmap_subscription_;
};

}

#endif  // D2__COSTMAP_POLYGON_GENERATOR__COSTMAP_POLYGON_GENERATOR_NODE_HPP_