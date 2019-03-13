#include "mutable_transform_publisher/publisher.h"
#include <rclcpp/rclcpp.hpp>

mutable_transform_publisher::Publisher::Publisher(const std::string& source,
                                                  const std::string& target,
                                                  const std::chrono::seconds& period,
                                                  const geometry_msgs::msg::Transform& init_tf,
                                                  tf2_ros::TransformBroadcaster& broadcaster)
  : Node("publisher")
  , source_(source)
  , target_(target)
  , broadcaster_(broadcaster)
{
  tf_.transform = init_tf;
  tf_.header.frame_id = source;
  tf_.child_frame_id = target;

//  ros::NodeHandle nh;
  pub_timer_ = this -> create_wall_timer(period,
                                         [this]() {
      tf_.header.stamp = rclcpp::Node::now();
      broadcaster_.sendTransform(tf_);
  });
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

//void mutable_transform_publisher::Publisher::onPublishTimeout(const ros::TimerEvent& e)
//{
//  tf_.header.stamp = e.current_real;
//  broadcaster_.sendTransform(tf_);
//}
