#include "foundation_common/base_component.hpp"
// Note: Add yaml-cpp dependency for configuration parsing
// For now, we'll use a simple string-based configuration

namespace foundation_common
{

BaseComponent::BaseComponent(const std::string & node_name, const std::string & component_type)
: Node(node_name), component_type_(component_type)
{
  // Initialize status
  status_.component_name = node_name;
  status_.component_type = component_type_;
  status_.status = foundation_interfaces::msg::ComponentStatus::OFFLINE;
  status_.error_message = "";
  status_.last_update_time = this->now().seconds();

  // Create status publisher
  status_publisher_ = this->create_publisher<foundation_interfaces::msg::ComponentStatus>(
    "~/status", 10);

  // Create configuration service
  configure_service_ = this->create_service<foundation_interfaces::srv::ConfigureComponent>(
    "~/configure",
    std::bind(&BaseComponent::configure_callback, this,
    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "BaseComponent '%s' of type '%s' initialized", 
              node_name.c_str(), component_type_.c_str());
}

bool BaseComponent::configure(const std::string & config_yaml)
{
  try {
    // Simple configuration parsing (replace with yaml-cpp when available)
    RCLCPP_INFO(this->get_logger(), "Configuration received: %s", config_yaml.c_str());
    // TODO: Parse YAML configuration when yaml-cpp is added as dependency
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse configuration: %s", e.what());
    return false;
  }
}

foundation_interfaces::msg::ComponentStatus BaseComponent::get_status() const
{
  return status_;
}

void BaseComponent::set_status(uint8_t status, const std::string & error_message)
{
  status_.status = status;
  status_.error_message = error_message;
  status_.last_update_time = this->now().seconds();
  status_.header.stamp = this->now();
  
  publish_status();
}

void BaseComponent::publish_status()
{
  status_.header.stamp = this->now();
  status_.last_update_time = this->now().seconds();
  status_publisher_->publish(status_);
}

void BaseComponent::start_status_timer()
{
  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&BaseComponent::publish_status, this));
}

void BaseComponent::configure_callback(
  const std::shared_ptr<foundation_interfaces::srv::ConfigureComponent::Request> request,
  std::shared_ptr<foundation_interfaces::srv::ConfigureComponent::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received configuration request for component: %s", 
              request->component_name.c_str());

  if (request->component_name != status_.component_name) {
    response->success = false;
    response->message = "Component name mismatch";
    return;
  }

  response->success = configure(request->configuration_yaml);
  response->message = response->success ? "Configuration applied successfully" : "Configuration failed";
  response->component_status = get_status();
}

} // namespace foundation_common
