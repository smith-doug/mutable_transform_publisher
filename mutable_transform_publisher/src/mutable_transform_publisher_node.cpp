#include <rclcpp/rclcpp.hpp>

#include "mutable_transform_publisher/mutable_transform_publisher.h"
#include "mutable_transform_publisher/yaml_serialization.h"

bool loadAndAddPublishers(const std::string& yaml_path,
                          mutable_transform_publisher::MutableTransformPublisher& pub)
{
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  if (!mutable_transform_publisher::deserialize(yaml_path, tfs))
  {
       // TODO: Print error for ROS2
//    RCLCPP_ERROR(node->get_logger(), "Unable to add transform");
    return false;
  }

  for (const auto& t : tfs)
  {
    if (!pub.add(t, std::chrono::milliseconds(1000)))
    {
        // TODO: Print error for ROS2
//      RCLCPP_ERROR(node->get_logger(), "Unable to add transform");
      return false;
    }
  }
  return true;
}

bool savePublishers(const std::string& yaml_path,
                    const mutable_transform_publisher::MutableTransformPublisher& pub)
{
  const auto new_tfs = pub.getAllTransforms();

  if (!mutable_transform_publisher::serialize(yaml_path, new_tfs))
  {
    std::cerr << "mutable_transform_publisher: Unable to serialize transforms to " << yaml_path << "\n";
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mutable_tf_publisher");

  node->declare_parameter("yaml_path");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  bool yaml_specified = false;
  std::string yaml_path_param;
  try {
      yaml_path_param = parameters_client->get_parameter<std::string>("yaml_path");
      yaml_specified = true;
  } catch (std::runtime_error &e) {
      RCLCPP_INFO(node->get_logger(), "yaml_path param not set");
  }
  bool commit = parameters_client->get_parameter<bool>("commit", true);

  // Create the publisher
  mutable_transform_publisher::MutableTransformPublisher pub(node);

  if (yaml_specified)
  {
    if (!loadAndAddPublishers(yaml_path_param, pub)) return 1;
    RCLCPP_INFO(node->get_logger(), "Added publishers");
  }

  rclcpp::spin(node);

  if (yaml_specified && commit)
  {
    if (!savePublishers(yaml_path_param, pub)) return 2;
    RCLCPP_INFO(node->get_logger(), "Saving updated yaml config");
  }

  rclcpp::shutdown();
  return 0;
}
