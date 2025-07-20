#ifndef FOUNDATION_COMMON_BASE_COMPONENT_HPP_
#define FOUNDATION_COMMON_BASE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <foundation_interfaces/msg/component_status.hpp>
#include <foundation_interfaces/srv/configure_component.hpp>
#include <std_msgs/msg/header.hpp>

namespace foundation_common
{

class BaseComponent : public rclcpp::Node
{
public:
  explicit BaseComponent(const std::string & node_name, const std::string & component_type);
  virtual ~BaseComponent() = default;

  // Component lifecycle methods
  virtual bool initialize() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool shutdown() = 0;

  // Configuration
  virtual bool configure(const std::string & config_yaml);

  // Status management
  foundation_interfaces::msg::ComponentStatus get_status() const;
  void set_status(uint8_t status, const std::string & error_message = "");

protected:
  // Status publishing
  void publish_status();
  void start_status_timer();

  // Configuration service callback
  void configure_callback(
    const std::shared_ptr<foundation_interfaces::srv::ConfigureComponent::Request> request,
    std::shared_ptr<foundation_interfaces::srv::ConfigureComponent::Response> response);

  std::string component_type_;
  foundation_interfaces::msg::ComponentStatus status_;

private:
  rclcpp::Publisher<foundation_interfaces::msg::ComponentStatus>::SharedPtr status_publisher_;
  rclcpp::Service<foundation_interfaces::srv::ConfigureComponent>::SharedPtr configure_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace foundation_common

#endif // FOUNDATION_COMMON_BASE_COMPONENT_HPP_
