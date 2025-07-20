# Adding New Components

This guide shows you how to extend the ROS2 Robotics Foundation by adding new components like robot arms, sensors, actuators, and other robotic modules.

## Component Types

The foundation supports several types of components:

- **Sensors**: Cameras, LiDAR, IMU, encoders, etc.
- **Actuators**: Motors, servos, grippers, etc.
- **Controllers**: Motion controllers, behavior controllers
- **Processors**: Computer vision, AI inference, path planning
- **Hardware Interfaces**: Device drivers, communication modules

## Quick Start: Adding a Camera Component

Let's walk through adding a camera component step by step.

### 1. Create the Package

```bash
cd src/
ros2 pkg create --build-type ament_cmake camera_component \
    --dependencies rclcpp sensor_msgs image_transport foundation_common foundation_interfaces
```

### 2. Implement the Component (C++)

Create `src/camera_component/src/camera_component.cpp`:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <foundation_common/base_component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

class CameraComponent : public foundation_common::BaseComponent
{
public:
    CameraComponent() : BaseComponent("camera_component", "sensor")
    {
        // Initialize image transport
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Create publisher
        image_pub_ = image_transport_->advertise("~/image", 1);
        
        // Declare parameters
        this->declare_parameter("device_id", 0);
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("frame_id", "camera_link");
    }
    
    bool initialize() override
    {
        RCLCPP_INFO(this->get_logger(), "Initializing camera component...");
        
        // Get parameters
        device_id_ = this->get_parameter("device_id").as_int();
        frame_rate_ = this->get_parameter("frame_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Initialize camera (mock implementation)
        camera_initialized_ = true;
        
        // Set status
        set_status(foundation_interfaces::msg::ComponentStatus::ONLINE);
        
        RCLCPP_INFO(this->get_logger(), "Camera component initialized");
        return true;
    }
    
    bool start() override
    {
        RCLCPP_INFO(this->get_logger(), "Starting camera component...");
        
        // Start capture timer
        capture_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate_)),
            std::bind(&CameraComponent::capture_and_publish, this)
        );
        
        start_status_timer();
        return true;
    }
    
    bool stop() override
    {
        RCLCPP_INFO(this->get_logger(), "Stopping camera component...");
        capture_timer_.reset();
        return true;
    }
    
    bool shutdown() override
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down camera component...");
        camera_initialized_ = false;
        set_status(foundation_interfaces::msg::ComponentStatus::OFFLINE);
        return true;
    }

private:
    void capture_and_publish()
    {
        if (!camera_initialized_) return;
        
        // Create mock image message
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id_;
        msg->height = 480;
        msg->width = 640;
        msg->encoding = "rgb8";
        msg->step = msg->width * 3;
        msg->data.resize(msg->height * msg->step);
        
        // Fill with mock data (gradient pattern)
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            msg->data[i] = (i / 3) % 256;     // R
            msg->data[i + 1] = (i / 6) % 256; // G  
            msg->data[i + 2] = (i / 9) % 256; // B
        }
        
        image_pub_.publish(msg);
    }
    
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    
    int device_id_;
    double frame_rate_;
    std::string frame_id_;
    bool camera_initialized_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraComponent>();
    
    // Initialize and start the component
    if (node->initialize() && node->start()) {
        rclcpp::spin(node);
        node->stop();
        node->shutdown();
    }
    
    rclcpp::shutdown();
    return 0;
}
```

### 3. Update CMakeLists.txt

```cmake
# Add to CMakeLists.txt
add_executable(camera_component src/camera_component.cpp)
ament_target_dependencies(camera_component
    rclcpp
    sensor_msgs
    image_transport
    foundation_common
    foundation_interfaces
)

install(TARGETS camera_component
    DESTINATION lib/${PROJECT_NAME}
)
```

### 4. Create Launch File

Create `launch/camera.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_component',
            executable='camera_component',
            name='camera',
            parameters=[{
                'device_id': 0,
                'frame_rate': 30.0,
                'frame_id': 'camera_link'
            }],
            output='screen'
        )
    ])
```

### 5. Add to System Launch

Update `foundation_bringup/launch/basic_system.launch.py`:

```python
# Add camera component
camera_node = Node(
    package='camera_component',
    executable='camera_component',
    name='camera',
    parameters=[{
        'device_id': 0,
        'frame_rate': 30.0,
        'frame_id': 'camera_link'
    }],
    output='screen'
)

# Add to LaunchDescription
return LaunchDescription([
    # ... existing nodes ...
    camera_node,
])
```

## Python Component Example

Here's how to create the same camera component in Python:

### 1. Create Python Node

Create `scripts/camera_component.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foundation_interfaces.msg import ComponentStatus
from sensor_msgs.msg import Image
import numpy as np

class CameraComponent(Node):
    def __init__(self):
        super().__init__('camera_component')
        
        # Component status
        self.status = ComponentStatus()
        self.status.component_name = 'camera_component'
        self.status.component_type = 'sensor'
        self.status.status = ComponentStatus.OFFLINE
        
        # Publishers
        self.status_pub = self.create_publisher(ComponentStatus, '~/status', 10)
        self.image_pub = self.create_publisher(Image, '~/image', 10)
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        
        self.device_id = self.get_parameter('device_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize
        self.initialize_component()
        
    def initialize_component(self):
        """Initialize the camera component."""
        self.get_logger().info('Initializing camera component...')
        
        # Mock initialization
        self.camera_initialized = True
        
        # Set status
        self.status.status = ComponentStatus.ONLINE
        self.status.error_message = ""
        
        # Start timers
        self.capture_timer = self.create_timer(
            1.0 / self.frame_rate, 
            self.capture_and_publish
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Camera component initialized')
    
    def capture_and_publish(self):
        """Capture and publish camera image."""
        if not self.camera_initialized:
            return
            
        # Create mock image
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        
        # Generate mock data
        data = np.random.randint(0, 256, (msg.height, msg.width, 3), dtype=np.uint8)
        msg.data = data.flatten().tolist()
        
        self.image_pub.publish(msg)
    
    def publish_status(self):
        """Publish component status."""
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status.last_update_time = self.get_clock().now().nanoseconds / 1e9
        self.status_pub.publish(self.status)

def main(args=None):
    rclpy.init(args=args)
    node = CameraComponent()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Component Features

### 1. Configuration Service

Add configuration support to your component:

```cpp
// In your component class
rclcpp::Service<foundation_interfaces::srv::ConfigureComponent>::SharedPtr config_service_;

// In constructor
config_service_ = this->create_service<foundation_interfaces::srv::ConfigureComponent>(
    "~/configure",
    std::bind(&MyComponent::configure_callback, this, std::placeholders::_1, std::placeholders::_2)
);

// Implementation
void configure_callback(
    const std::shared_ptr<foundation_interfaces::srv::ConfigureComponent::Request> request,
    std::shared_ptr<foundation_interfaces::srv::ConfigureComponent::Response> response)
{
    // Parse YAML configuration
    try {
        YAML::Node config = YAML::Load(request->configuration_yaml);
        
        // Apply configuration
        if (config["frame_rate"]) {
            frame_rate_ = config["frame_rate"].as<double>();
        }
        
        response->success = true;
        response->message = "Configuration applied successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Configuration failed: ") + e.what();
    }
}
```

### 2. Mode Switching

Add mode switching capabilities:

```cpp
// Add mode service
rclcpp::Service<foundation_interfaces::srv::SetMode>::SharedPtr mode_service_;

// In constructor
mode_service_ = this->create_service<foundation_interfaces::srv::SetMode>(
    "~/set_mode",
    std::bind(&MyComponent::set_mode_callback, this, std::placeholders::_1, std::placeholders::_2)
);

// Implementation  
void set_mode_callback(
    const std::shared_ptr<foundation_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<foundation_interfaces::srv::SetMode::Response> response)
{
    switch (request->mode) {
        case foundation_interfaces::srv::SetMode::Request::AUTO:
            // Switch to automatic mode
            response->success = true;
            response->message = "Switched to AUTO mode";
            break;
        case foundation_interfaces::srv::SetMode::Request::MANUAL:
            // Switch to manual mode
            response->success = true;
            response->message = "Switched to MANUAL mode";
            break;
        default:
            response->success = false;
            response->message = "Unsupported mode";
    }
}
```

### 3. Action Server

For long-running operations, implement action servers:

```cpp
#include <rclcpp_action/rclcpp_action.hpp>
#include <foundation_interfaces/action/execute_task.hpp>

// Add action server
rclcpp_action::Server<foundation_interfaces::action::ExecuteTask>::SharedPtr action_server_;

// In constructor
action_server_ = rclcpp_action::create_server<foundation_interfaces::action::ExecuteTask>(
    this,
    "~/execute_task",
    std::bind(&MyComponent::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MyComponent::handle_cancel, this, std::placeholders::_1),
    std::bind(&MyComponent::handle_accepted, this, std::placeholders::_1)
);
```

## Testing Your Component

### 1. Unit Tests

Create `test/test_camera_component.cpp`:

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "camera_component/camera_component.hpp"

class CameraComponentTest : public ::testing::Test
{
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<CameraComponent>();
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::shared_ptr<CameraComponent> node_;
};

TEST_F(CameraComponentTest, Initialization) {
    EXPECT_TRUE(node_->initialize());
}

TEST_F(CameraComponentTest, StartStop) {
    ASSERT_TRUE(node_->initialize());
    EXPECT_TRUE(node_->start());
    EXPECT_TRUE(node_->stop());
}
```

### 2. Integration Tests

Test your component with the system:

```bash
# Build and source workspace
colcon build && source install/setup.bash

# Launch your component
ros2 launch camera_component camera.launch.py

# Test topics and services
ros2 topic list | grep camera
ros2 service list | grep camera
ros2 topic echo /camera/status
```

## Component Templates

The foundation includes templates for common component types:

```bash
# Generate a new sensor component
./scripts/generate_component.sh --type sensor --name my_sensor --language cpp

# Generate a new actuator component  
./scripts/generate_component.sh --type actuator --name my_actuator --language python

# Generate a new controller component
./scripts/generate_component.sh --type controller --name my_controller --language cpp
```

## Best Practices

1. **Always inherit from BaseComponent** for consistency
2. **Implement proper error handling** and status reporting
3. **Use parameters for configuration** instead of hardcoded values
4. **Add comprehensive logging** for debugging
5. **Include unit and integration tests**
6. **Document your component's interfaces**
7. **Follow ROS2 naming conventions**
8. **Use appropriate QoS profiles** for your data

## Next Steps

- Add your component to the system launch files
- Create visualization configs for RViz
- Add simulation support if applicable
- Write documentation and examples
- Consider contributing back to the foundation

With this guide, you should be able to add any type of component to the ROS2 Robotics Foundation!
