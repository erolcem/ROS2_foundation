#ifndef FOUNDATION_COMMON_TF_HELPER_HPP_
#define FOUNDATION_COMMON_TF_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vector>
#include <string>

namespace foundation_common
{

/**
 * @brief Helper class for TF2 operations
 * 
 * This class provides convenient methods for common TF2 operations
 * including transform lookups, coordinate transformations, and publishing.
 */
class TfHelper
{
public:
  /**
   * @brief Constructor with shared pointer to node
   * @param node Shared pointer to ROS2 node
   */
  explicit TfHelper(const rclcpp::Node::SharedPtr& node);
  
  /**
   * @brief Constructor with raw pointer to node
   * @param node Raw pointer to ROS2 node
   */
  explicit TfHelper(rclcpp::Node* node);

  /**
   * @brief Look up transform between two frames
   * @param target_frame Target frame ID
   * @param source_frame Source frame ID
   * @param transform Output transform
   * @param timeout Timeout for lookup
   * @return True if transform found, false otherwise
   */
  bool lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::msg::TransformStamped& transform,
    const rclcpp::Duration& timeout = rclcpp::Duration::from_nanoseconds(0));

  /**
   * @brief Transform pose from one frame to another
   * @param pose_in Input pose
   * @param pose_out Output pose
   * @param target_frame Target frame ID
   * @param timeout Timeout for transform
   * @return True if transform successful, false otherwise
   */
  bool transformPose(
    const geometry_msgs::msg::PoseStamped& pose_in,
    geometry_msgs::msg::PoseStamped& pose_out,
    const std::string& target_frame,
    const rclcpp::Duration& timeout = rclcpp::Duration::from_nanoseconds(0));

  /**
   * @brief Transform point from one frame to another
   * @param point_in Input point
   * @param point_out Output point
   * @param target_frame Target frame ID
   * @param timeout Timeout for transform
   * @return True if transform successful, false otherwise
   */
  bool transformPoint(
    const geometry_msgs::msg::PointStamped& point_in,
    geometry_msgs::msg::PointStamped& point_out,
    const std::string& target_frame,
    const rclcpp::Duration& timeout = rclcpp::Duration::from_nanoseconds(0));

  /**
   * @brief Publish static transform
   * @param transform Transform to publish
   */
  void publishStaticTransform(const geometry_msgs::msg::TransformStamped& transform);

  /**
   * @brief Publish dynamic transform
   * @param transform Transform to publish
   */
  void publishTransform(const geometry_msgs::msg::TransformStamped& transform);

  /**
   * @brief Publish multiple dynamic transforms
   * @param transforms Vector of transforms to publish
   */
  void publishTransforms(const std::vector<geometry_msgs::msg::TransformStamped>& transforms);

  /**
   * @brief Create a transform message
   * @param parent_frame Parent frame ID
   * @param child_frame Child frame ID
   * @param x Translation x
   * @param y Translation y
   * @param z Translation z
   * @param qx Rotation quaternion x
   * @param qy Rotation quaternion y
   * @param qz Rotation quaternion z
   * @param qw Rotation quaternion w
   * @param timestamp Timestamp for transform
   * @return Transform message
   */
  geometry_msgs::msg::TransformStamped createTransform(
    const std::string& parent_frame,
    const std::string& child_frame,
    double x, double y, double z,
    double qx, double qy, double qz, double qw,
    const rclcpp::Time& timestamp);

  /**
   * @brief Check if a frame exists in the TF tree
   * @param frame_id Frame ID to check
   * @return True if frame exists, false otherwise
   */
  bool frameExists(const std::string& frame_id);

  /**
   * @brief Get all frame names in the TF tree
   * @return Vector of frame names
   */
  std::vector<std::string> getAllFrameNames();

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms_;
};

}  // namespace foundation_common

#endif  // FOUNDATION_COMMON_TF_HELPER_HPP_
