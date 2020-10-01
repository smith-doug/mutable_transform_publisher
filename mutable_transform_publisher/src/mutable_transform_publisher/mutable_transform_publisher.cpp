#include "mutable_transform_publisher/mutable_transform_publisher.h"
#include "mutable_transform_publisher/yaml_serialization.h"

static bool isNormalized(const geometry_msgs::msg::Quaternion& q, const double eps = 1e-6)
{
  const auto sum_sq = sqrt(pow(q.w, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2));
  return std::abs(1.0 - sum_sq) < eps;
}

mutable_transform_publisher::MutableTransformPublisher::MutableTransformPublisher(rclcpp::Node::SharedPtr node, const std::string& yaml_path, const double& period, const bool& commit)
  : node_(node)
  , broadcaster_(node)
  , set_transform_server_(node -> create_service<mutable_transform_publisher_msgs::srv::SetTransform>("set_transform", std::bind(&MutableTransformPublisher::setTransformCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
  , yaml_path_(yaml_path)
  , period_(std::chrono::duration<double>(period))
  , commit_(commit)
{
  loadAndAddPublishers(yaml_path_);
}

bool mutable_transform_publisher::MutableTransformPublisher::add(const geometry_msgs::msg::TransformStamped& transform,
                                                                 const std::chrono::duration<double>& period)
{
  if (!validate(transform))
  {
    RCLCPP_WARN(node_->get_logger(), "Transform push rejected");
    return false;
  }

  const auto& source = transform.header.frame_id;
  const auto target = transform.child_frame_id;
  const auto key = makeKey(source, target);
  std::unique_ptr<Publisher> pub (new Publisher(source, target, period, transform.transform, broadcaster_, node_));
  const auto r = pub_map_.emplace(key, std::move(pub));
  return r.second;
}

bool mutable_transform_publisher::MutableTransformPublisher::loadAndAddPublishers(const std::string& yaml_path)
{
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  if (!mutable_transform_publisher::deserialize(yaml_path, tfs))
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to add transform");
    return false;
  }

  for (const auto& t : tfs)
  {
    if (!this->add(t, std::chrono::duration<double>(period_)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to add transform");
      return false;
    }
  }
  return true;
}

bool mutable_transform_publisher::MutableTransformPublisher::savePublishers(const std::string& yaml_path)
{
  const auto new_tfs = this->getAllTransforms();

  if (!mutable_transform_publisher::serialize(yaml_path, new_tfs))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to serialize transforms to path: " + yaml_path);
    return false;
  }
  return true;
}

std::vector<geometry_msgs::msg::TransformStamped>
mutable_transform_publisher::MutableTransformPublisher::getAllTransforms() const
{
  std::vector<geometry_msgs::msg::TransformStamped> result;
  result.reserve(pub_map_.size());

  for (const auto& k : pub_map_)
  {
    result.push_back(k.second->getTransform());
  }
  return result;
}

bool mutable_transform_publisher::MutableTransformPublisher::setTransformCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                                                  const std::shared_ptr<mutable_transform_publisher_msgs::srv::SetTransform::Request> req,
                                                                                  std::shared_ptr<mutable_transform_publisher_msgs::srv::SetTransform::Response> res)
{
  (void)request_header;

  auto* pub = findPublisher(req->transform.header.frame_id, req->transform.child_frame_id);
  if (pub)
  {
    res->was_replaced = true;
    res->old_transform = pub->setTransform(req->transform.transform);
  }
  else
  {
    res->was_replaced = false;
    add(req->transform, period_);
  }

  if (commit_)
  {
    if (!this->savePublishers(yaml_path_))
      RCLCPP_ERROR(node_->get_logger(), "Failed to serialize transforms to path: " + yaml_path_);

  }

  return true;
}

mutable_transform_publisher::Publisher*
mutable_transform_publisher::MutableTransformPublisher::findPublisher(const std::string& source,
                                                                      const std::string& target) const
{
  const auto key = makeKey(source, target);
  auto it = pub_map_.find(key);
  if (it == pub_map_.end())
  {
    return nullptr;
  }
  else
  {
    return it->second.get();
  }
}

std::string mutable_transform_publisher::MutableTransformPublisher::makeKey(const std::string& source,
                                                                            const std::string& target) const
{
  return source + target;
}

bool mutable_transform_publisher::MutableTransformPublisher::validate(const geometry_msgs::msg::TransformStamped& t) const
{
  if (t.child_frame_id.empty())
  {
      RCLCPP_WARN(node_->get_logger(), "transform's child_frame_id is empty");
      return false;
  }

  if (t.header.frame_id.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "transform's header.frame_id is empty");
    return false;
  }

  if (!isNormalized(t.transform.rotation))
  {
    RCLCPP_WARN(node_->get_logger(), "transform quaternion is not normalized");
    return false;
  }  
  return true;
}
