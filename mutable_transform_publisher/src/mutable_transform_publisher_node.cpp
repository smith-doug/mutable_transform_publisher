#include <rclcpp/rclcpp.hpp>

#include "mutable_transform_publisher/mutable_transform_publisher.h"
#include "mutable_transform_publisher/yaml_serialization.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mutable_tf_publisher");

  node->declare_parameter("yaml_path");
  node->declare_parameter("period");

  double period;
  node->get_parameter_or<double>("period", period, 0.1);

  bool yaml_specified = false;
  std::string yaml_path_param;
  try {
    node->get_parameter<std::string>("yaml_path", yaml_path_param);
    yaml_specified = true;
  } catch (std::runtime_error & e) {
    RCLCPP_INFO(node->get_logger(), "yaml_path param not set");
  }

  if (!yaml_specified) {
    RCLCPP_ERROR(node->get_logger(), "required param yaml_path not found: aborting");
    return -1;
  }

  bool commit;
  node->get_parameter_or<bool>("commit", commit, true);

  // Create the publisher
  mutable_transform_publisher::MutableTransformPublisher pub(node, yaml_path_param, period, commit);

  rclcpp::spin(node);

  if (yaml_specified && commit) {
    if (!pub.savePublishers(yaml_path_param)) {return 2;}
    RCLCPP_INFO(node->get_logger(), "Saving updated yaml config");
  }

  rclcpp::shutdown();
  return 0;
}
