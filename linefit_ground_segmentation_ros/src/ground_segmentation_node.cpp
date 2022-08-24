#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode : public rclcpp::Node
{
 public:
  SegmentationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("SegmentationNode", options)
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
  {

    RCLCPP_INFO(get_logger(), "Running SegmentationNode");
    params_ = std::make_unique<GroundSegmentationParams>();
    params_->n_bins = declare_parameter("n_bins", 30);
    params_->n_segments = declare_parameter("n_segments", 180);
    params_->max_dist_to_line = declare_parameter("max_dist_to_line", 0.15);
    params_->max_slope = declare_parameter("max_slope", 1);
    params_->min_slope = declare_parameter("min_slope", 0);
    params_->long_threshold = declare_parameter("long_threshold", 2.0);
    params_->max_long_height = declare_parameter("max_long_height", 0.1);
    params_->max_start_height = declare_parameter("max_start_height", 0.2);
    params_->sensor_height = declare_parameter("sensor_height", 0.2);
    params_->line_search_angle = declare_parameter("line_search_angle", 0.2);
    params_->n_threads = declare_parameter("n_threads", 4);

    //params_.r_min = declare_parameter("r_min", 0.09);
    //params_.r_max = declare_parameter("r_max", 400);
    //params_.max_fit_error = declare_parameter("max_fit_error", 0.01);

    segmenter_ = std::make_unique<GroundSegmentation>(*params_);

    input_topic_ = declare_parameter("input_topic", "scan_cloud");
    ground_output_topic_ = declare_parameter("ground_output_topic", "ground_cloud");
    obstacle_output_topic_ = declare_parameter("obstacle_output_topic", "obstacle_cloud");

    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_output_topic_, 10);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ground_output_topic_, 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10,
      std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1)
    );
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *cloud);

    std::vector<int> labels;
    segmenter_->segment(*cloud, &labels);

    pcl::PointCloud<pcl::PointXYZ> ground_cloud;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
      if (labels[i] == 1)
      {
        ground_cloud.push_back(cloud->points[i]);
      }
      else
      {
        obstacle_cloud.push_back(cloud->points[i]);
      }
    }
    sensor_msgs::msg::PointCloud2 ground_cloud_msg;
    pcl::toROSMsg(ground_cloud, ground_cloud_msg);
    ground_cloud_msg.header = cloud_msg->header;

    sensor_msgs::msg::PointCloud2 obstacle_cloud_msg;
    pcl::toROSMsg(obstacle_cloud, obstacle_cloud_msg);
    obstacle_cloud_msg.header = cloud_msg->header;

    ground_pub_->publish(ground_cloud_msg);
    obstacle_pub_->publish(obstacle_cloud_msg);
  }

private:

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string gravity_aligned_frame_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

  std::string input_topic_;
  std::string ground_output_topic_;
  std::string obstacle_output_topic_;
  bool latch_;

  std::unique_ptr<GroundSegmentationParams> params_;
  std::unique_ptr<GroundSegmentation> segmenter_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // Start node.
  std::shared_ptr<SegmentationNode> node = std::make_shared<SegmentationNode>(options);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
