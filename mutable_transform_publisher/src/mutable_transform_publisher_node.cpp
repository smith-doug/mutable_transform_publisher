#include <rclcpp/rclcpp.hpp>

#include "mutable_transform_publisher/mutable_transform_publisher.h"
#include "mutable_transform_publisher/yaml_serialization.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mutable_tf_publisher");

  node->declare_parameter<std::string>("yaml_path", "");
  node->declare_parameter<double>("period", 0.1);
  node->declare_parameter<bool>("commit", true);

  double period;
  node->get_parameter("period", period);

  bool commit;
  node->get_parameter<bool>("commit", commit);

  bool yaml_specified = false;
  std::string yaml_path_param;

  node->get_parameter<std::string>("yaml_path", yaml_path_param);
  yaml_specified = yaml_path_param != "";

  if (!yaml_specified) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Param yaml_path not found.  Will not save or load transforms");
    commit = false;
  }

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
