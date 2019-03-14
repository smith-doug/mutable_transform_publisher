#include "mutable_transform_publisher/publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <chrono>

mutable_transform_publisher::Publisher::Publisher(const std::string& source,
                                                  const std::string& target,
                                                  const std::chrono::milliseconds& period,
                                                  const geometry_msgs::msg::Transform& init_tf,
                                                  tf2_ros::TransformBroadcaster& broadcaster,
                                                  std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  , source_(source)
  , target_(target)
  , broadcaster_(broadcaster)
{
  tf_.transform = init_tf;
  tf_.header.frame_id = source;
  tf_.child_frame_id = target;
  std::printf("making a publisher\n");

  pub_timer_ = node_ -> create_wall_timer(std::chrono::seconds(1), std::bind(&Publisher::onPublishTimeout, this));
//  auto thread_fn = [this]() -> void {rclcpp::spin(node_);};
//  static std::thread thread(thread_fn);
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
  std::printf("callback\n");
  tf_.header.stamp = node_->now();
  broadcaster_.sendTransform(tf_);
}

//void mutable_transform_publisher::Publisher::onPublishTimeout(const ros::TimerEvent& e)
//{
//  tf_.header.stamp = e.current_real;
//  broadcaster_.sendTransform(tf_);
//}
