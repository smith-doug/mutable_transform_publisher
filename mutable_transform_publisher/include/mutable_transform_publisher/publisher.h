#ifndef MCT_PUBLISHER_H
#define MCT_PUBLISHER_H

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace mutable_transform_publisher
{

class Publisher
{
public:
  Publisher(const std::string& source,
            const std::string& target,
            const std::chrono::duration<double>& period,
            const geometry_msgs::msg::Transform& init_tf,
            tf2_ros::TransformBroadcaster& broadcaster,
            std::shared_ptr<rclcpp::Node> node);

  geometry_msgs::msg::TransformStamped setTransform(const geometry_msgs::msg::Transform& t);

  geometry_msgs::msg::TransformStamped getTransform() const;

private:
  void onPublishTimeout();

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::string source_;
  std::string target_;
  geometry_msgs::msg::TransformStamped tf_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  tf2_ros::TransformBroadcaster& broadcaster_;
};

}

#endif // MCT_PUBLISHER_H
