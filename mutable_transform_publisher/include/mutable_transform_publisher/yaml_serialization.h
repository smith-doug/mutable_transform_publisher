#ifndef MTP_YAML_SERIALIZATION_H
#define MTP_YAML_SERIALIZATION_H

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mutable_transform_publisher
{

bool deserialize(const std::string & path, std::vector<geometry_msgs::msg::TransformStamped> & tfs);

bool serialize(
  const std::string & path,
  const std::vector<geometry_msgs::msg::TransformStamped> & tfs);

} // namespace mutable_transform_publisher

#endif // MTP_YAML_SERIALIZATION_H
