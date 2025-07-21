#include "foundation_common/tf_helper.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace foundation_common
{

TfHelper::TfHelper(const rclcpp::Node::SharedPtr& node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(node),
  static_tf_broadcaster_(node)
{
}

TfHelper::TfHelper(rclcpp::Node* node)
: node_(node->shared_from_this()),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(node),
  static_tf_broadcaster_(node)
{
}

bool TfHelper::lookupTransform(
  const std::string& target_frame,
  const std::string& source_frame,
  geometry_msgs::msg::TransformStamped& transform,
  const rclcpp::Duration& timeout)
{
  try {
    transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, rclcpp::Time(0), timeout);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

bool TfHelper::transformPose(
  const geometry_msgs::msg::PoseStamped& pose_in,
  geometry_msgs::msg::PoseStamped& pose_out,
  const std::string& target_frame,
  const rclcpp::Duration& timeout)
{
  try {
    pose_out = tf_buffer_.transform(pose_in, target_frame, timeout);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform pose to %s: %s",
                target_frame.c_str(), ex.what());
    return false;
  }
}

bool TfHelper::transformPoint(
  const geometry_msgs::msg::PointStamped& point_in,
  geometry_msgs::msg::PointStamped& point_out,
  const std::string& target_frame,
  const rclcpp::Duration& timeout)
{
  try {
    point_out = tf_buffer_.transform(point_in, target_frame, timeout);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform point to %s: %s",
                target_frame.c_str(), ex.what());
    return false;
  }
}

void TfHelper::publishStaticTransform(
  const geometry_msgs::msg::TransformStamped& transform)
{
  static_transforms_.push_back(transform);
  static_tf_broadcaster_.sendTransform(static_transforms_);
}

void TfHelper::publishTransform(
  const geometry_msgs::msg::TransformStamped& transform)
{
  tf_broadcaster_.sendTransform(transform);
}

void TfHelper::publishTransforms(
  const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
{
  tf_broadcaster_.sendTransform(transforms);
}

geometry_msgs::msg::TransformStamped TfHelper::createTransform(
  const std::string& parent_frame,
  const std::string& child_frame,
  double x, double y, double z,
  double qx, double qy, double qz, double qw,
  const rclcpp::Time& timestamp)
{
  geometry_msgs::msg::TransformStamped transform;
  
  transform.header.stamp = timestamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  
  transform.transform.rotation.x = qx;
  transform.transform.rotation.y = qy;
  transform.transform.rotation.z = qz;
  transform.transform.rotation.w = qw;
  
  return transform;
}

bool TfHelper::frameExists(const std::string& frame_id)
{
  return tf_buffer_.canTransform("base_link", frame_id, rclcpp::Time(0));
}

std::vector<std::string> TfHelper::getAllFrameNames()
{
  return tf_buffer_.getAllFrameNames();
}

}  // namespace foundation_common
