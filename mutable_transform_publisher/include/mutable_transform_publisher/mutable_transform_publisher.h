#ifndef MCT_MUTABLE_TRANSFORM_PUBLISHER_H
#define MCT_MUTABLE_TRANSFORM_PUBLISHER_H
#include <rclcpp/rclcpp.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "mutable_transform_publisher/publisher.h"
#include "mutable_transform_publisher_msgs/srv/set_transform.hpp"

#include <memory>
#include <math.h>

namespace mutable_transform_publisher
{

class MutableTransformPublisher
{
public:
  MutableTransformPublisher(rclcpp::Node::SharedPtr node, const std::string& yaml_path, const bool& commit = true);

  bool add(const geometry_msgs::msg::TransformStamped& transform, const std::chrono::milliseconds& period);

  std::vector<geometry_msgs::msg::TransformStamped> getAllTransforms() const;

  bool savePublishers(const std::string& yaml_path);

private:
  bool loadAndAddPublishers(const std::string& yaml_path);

  bool setTransformCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<mutable_transform_publisher_msgs::srv::SetTransform::Request> req,
                            std::shared_ptr<mutable_transform_publisher_msgs::srv::SetTransform::Response> res);

  Publisher* findPublisher(const std::string& source, const std::string& target) const;

  std::string makeKey(const std::string& source, const std::string& target) const;

  bool validate(const geometry_msgs::msg::TransformStamped& t) const;

  std::shared_ptr<rclcpp::Node> node_;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Service<mutable_transform_publisher_msgs::srv::SetTransform>::SharedPtr set_transform_server_;
  std::map<std::string, std::unique_ptr<Publisher>> pub_map_;
  std::string yaml_path_;
  bool commit_;
};

}

#endif // MCT_MUTABLE_TRANSFORM_PUBLISHER_H
