#include <rclcpp/rclcpp.hpp>

#include "mutable_transform_publisher/mutable_transform_publisher.h"
#include "mutable_transform_publisher/yaml_serialization.h"

bool loadAndAddPublishers(const std::string& yaml_path,
                          mutable_transform_publisher::MutableTransformPublisher& pub)
{
  std::vector<geometry_msgs::TransformStamped> tfs;
  if (!mutable_transform_publisher::deserialize(yaml_path, tfs))
  {
//    ROS_ERROR_STREAM("Unable to parse yaml file at: " << yaml_path);
    return false;
  }

  for (const auto& t : tfs)
  {
    if (!pub.add(t, ros::Duration(1.0)))
    {
//      ROS_ERROR_STREAM("Unable to add transform");
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

//  ros::init(argc, argv, "mutable_tf_publisher", ros::init_options::AnonymousName);
//  ros::NodeHandle nh, pnh ("~");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

//  std::string yaml_path;
//  const bool yaml_specified = pnh.getParam("yaml_path", yaml_path);

  const bool yaml_specified = true;

  std::string yaml_path_param = parameters_client->get_parameter<std::string>("yaml_path");
  bool commit = parameters_client->get_parameter<bool>("commit", true);

//  const bool commit = pnh.param<bool>("commit", true);

  // Create the publisher
  mutable_transform_publisher::MutableTransformPublisher pub;

  if (yaml_specified)
  {
    if (!loadAndAddPublishers(yaml_path_param.value_to_string(), pub)) return 1;
  }

  rclcpp::spin();

  if (yaml_specified && commit)
  {
    if (!savePublishers(yaml_path_param.value_to_string(), pub)) return 2;
  }

  rclcpp::shutdown();
  return 0;
}
