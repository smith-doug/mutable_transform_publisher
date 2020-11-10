#include "mutable_transform_publisher/publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

mutable_transform_publisher::Publisher::Publisher(const std::string& source,
                                                  const std::string& target,
                                                  const std::chrono::duration<double>& period,
                                                  const geometry_msgs::msg::Transform& init_tf,
                                                  tf2_ros::TransformBroadcaster& broadcaster,
                                                  std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  , clock_(std::make_shared<rclcpp::Clock>())  // TODO: better to use node's clock, but it gives all zeros sometimes
  , source_(source)
  , target_(target)
  , broadcaster_(broadcaster)
{
  tf_.transform = init_tf;
  tf_.header.frame_id = source;
  tf_.child_frame_id = target;
  pub_timer_ = node_ -> create_wall_timer(period, std::bind(&Publisher::onPublishTimeout, this));
}

geometry_msgs::msg::TransformStamped mutable_transform_publisher::Publisher::setTransform(const geometry_msgs::msg::Transform& t)
{
  const auto copy = tf_;
  tf_.transform = t;
  return copy;
}

geometry_msgs::msg::TransformStamped mutable_transform_publisher::Publisher::getTransform() const
{
  return tf_;
}

void mutable_transform_publisher::Publisher::onPublishTimeout()
{
  tf_.header.stamp = clock_->now();
  broadcaster_.sendTransform(tf_);
}
