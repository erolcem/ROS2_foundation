# Tutorial 1: Your First Component

In this tutorial, you'll create your first component using the ROS2 Robotics Foundation. We'll build a simple temperature sensor component that publishes mock temperature data.

## Prerequisites

- Completed the [Getting Started](../getting_started.md) guide
- Basic understanding of ROS2 concepts
- Familiarity with Python or C++

## What You'll Learn

- How to create a new component package
- How to use the BaseComponent pattern
- How to publish sensor data
- How to integrate with the system monitor

## Step 1: Create the Package

First, let's create a new package for our temperature sensor:

```bash
cd src/
ros2 pkg create --build-type ament_cmake_python temperature_sensor \
    --dependencies rclpy foundation_interfaces foundation_common sensor_msgs
```

## Step 2: Implement the Temperature Sensor

Create the main sensor script at `src/temperature_sensor/temperature_sensor/temperature_sensor.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foundation_interfaces.msg import ComponentStatus
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header
import random
import math
import time

class TemperatureSensor(Node):
    """A simple temperature sensor component."""
    
    def __init__(self):
        super().__init__('temperature_sensor')
        
        # Component status
        self.status = ComponentStatus()
        self.status.component_name = 'temperature_sensor'
        self.status.component_type = 'sensor'
        self.status.status = ComponentStatus.OFFLINE
        
        # Publishers
        self.status_pub = self.create_publisher(ComponentStatus, '~/status', 10)
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        
        # Parameters
        self.declare_parameter('frame_id', 'temperature_link')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('base_temperature', 25.0)  # Celsius
        self.declare_parameter('noise_amplitude', 2.0)  # Celsius
        
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.base_temp = self.get_parameter('base_temperature').value
        self.noise_amp = self.get_parameter('noise_amplitude').value
        
        # Internal state
        self.sensor_online = False
        self.start_time = time.time()
        
        # Initialize component
        self.initialize_component()
    
    def initialize_component(self):
        """Initialize the temperature sensor."""
        self.get_logger().info('Initializing temperature sensor...')
        
        try:
            # Simulate sensor initialization
            time.sleep(0.5)  # Initialization delay
            
            # Set up timers
            self.temp_timer = self.create_timer(
                1.0 / self.publish_rate,
                self.publish_temperature
            )
            
            self.status_timer = self.create_timer(1.0, self.publish_status)
            
            # Mark as online
            self.sensor_online = True
            self.status.status = ComponentStatus.ONLINE
            self.status.error_message = ""
            
            self.get_logger().info('Temperature sensor initialized successfully')
            
        except Exception as e:
            self.status.status = ComponentStatus.ERROR
            self.status.error_message = str(e)
            self.get_logger().error(f'Failed to initialize temperature sensor: {e}')
    
    def publish_temperature(self):
        """Publish temperature reading."""
        if not self.sensor_online:
            return
        
        # Generate realistic temperature data with slow drift and noise
        current_time = time.time() - self.start_time
        
        # Slow sinusoidal drift (period: 60 seconds)
        drift = 3.0 * math.sin(2 * math.pi * current_time / 60.0)
        
        # Random noise
        noise = random.gauss(0, self.noise_amp)
        
        # Final temperature
        temperature = self.base_temp + drift + noise
        
        # Create temperature message
        msg = Temperature()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = temperature
        msg.variance = self.noise_amp ** 2  # Noise variance
        
        # Publish
        self.temp_pub.publish(msg)
        
        self.get_logger().debug(f'Published temperature: {temperature:.2f}°C')
    
    def publish_status(self):
        """Publish component status."""
        self.status.header = Header()
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status.last_update_time = self.get_clock().now().nanoseconds / 1e9
        
        # Update status based on sensor health
        if self.sensor_online:
            # Simulate occasional sensor hiccups
            if random.random() < 0.01:  # 1% chance
                self.status.status = ComponentStatus.ERROR
                self.status.error_message = "Temporary sensor communication error"
                self.sensor_online = False
        else:
            # Simulate recovery
            if random.random() < 0.2:  # 20% chance to recover
                self.status.status = ComponentStatus.ONLINE
                self.status.error_message = ""
                self.sensor_online = True
        
        self.status_pub.publish(self.status)

def main(args=None):
    rclpy.init(args=args)
    
    node = TemperatureSensor()
    
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

## Step 3: Update Package Configuration

Update `package.xml` to include the correct dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>temperature_sensor</name>
  <version>1.0.0</version>
  <description>Tutorial temperature sensor component</description>
  <maintainer email="your-email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>foundation_interfaces</depend>
  <depend>foundation_common</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Update `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(temperature_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/temperature_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Step 4: Create Launch File

Create `launch/temperature_sensor.launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for temperature sensor component."""
    
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='temperature_link',
        description='Frame ID for temperature sensor'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Temperature publishing rate in Hz'
    )
    
    base_temperature_arg = DeclareLaunchArgument(
        'base_temperature',
        default_value='22.0',
        description='Base temperature in Celsius'
    )
    
    # Get launch configurations
    frame_id = LaunchConfiguration('frame_id')
    publish_rate = LaunchConfiguration('publish_rate')
    base_temperature = LaunchConfiguration('base_temperature')
    
    # Temperature sensor node
    temperature_sensor_node = Node(
        package='temperature_sensor',
        executable='temperature_sensor.py',
        name='temperature_sensor',
        parameters=[{
            'frame_id': frame_id,
            'publish_rate': publish_rate,
            'base_temperature': base_temperature,
            'noise_amplitude': 1.5,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        frame_id_arg,
        publish_rate_arg,
        base_temperature_arg,
        temperature_sensor_node,
    ])
```

## Step 5: Build and Test

Now let's build and test our component:

```bash
# Build the workspace
cd /workspace  # or your workspace root
colcon build --packages-select temperature_sensor

# Source the workspace
source install/setup.bash

# Test the component
ros2 launch temperature_sensor temperature_sensor.launch.py
```

In another terminal:

```bash
# Check if the component is running
ros2 node list | grep temperature

# Check topics
ros2 topic list | grep temperature

# Monitor temperature data
ros2 topic echo /temperature_sensor/temperature

# Monitor component status
ros2 topic echo /temperature_sensor/status
```

## Step 6: Integration with System

To integrate with the foundation system, add your component to the main launch file.

Edit `src/foundation_bringup/launch/basic_system.launch.py`:

```python
# Add temperature sensor
temperature_sensor_node = Node(
    package='temperature_sensor',
    executable='temperature_sensor.py',
    name='temperature_sensor',
    parameters=[{
        'frame_id': 'temperature_link',
        'publish_rate': 2.0,
        'base_temperature': 23.0,
    }],
    output='screen'
)

# Add to the launch description
return LaunchDescription([
    # ... existing nodes ...
    temperature_sensor_node,
])
```

## Step 7: Verify System Integration

Launch the complete system:

```bash
ros2 launch foundation_bringup basic_system.launch.py
```

Check system health:

```bash
# Monitor overall system health
ros2 topic echo /system_health

# Check robot status
ros2 topic echo /robot_status
```

You should see your temperature sensor listed in the active components!

## Step 8: Visualize in RViz (Optional)

Add visualization to RViz:

1. Launch RViz: `rviz2`
2. Add a new display: Add → By topic → `/temperature_sensor/temperature` → Temperature
3. Configure the display properties as needed

## What You've Learned

✅ How to create a component that follows foundation patterns  
✅ How to publish sensor data with proper headers  
✅ How to integrate with the system monitoring  
✅ How to use parameters for configuration  
✅ How to handle component lifecycle and errors  

## Next Steps

- **Tutorial 2**: [Creating an Actuator Component](tutorial_02_actuator.md)
- **Tutorial 3**: [Building a Simple Robot Controller](tutorial_03_controller.md)
- **Advanced**: [Adding Your Component to Simulation](tutorial_04_simulation.md)

## Troubleshooting

**Component not showing in system health?**
- Check that the component publishes status on `~/status` topic
- Verify the component status message format

**Temperature data looks wrong?**
- Check the parameter values in the launch file
- Verify the frame_id matches your robot description

**Build errors?**
- Make sure all dependencies are installed
- Check that the Python script is executable: `chmod +x temperature_sensor.py`

Congratulations! You've created your first component using the ROS2 Robotics Foundation! 🎉
