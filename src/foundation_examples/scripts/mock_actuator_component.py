#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foundation_interfaces.msg import ComponentStatus
from foundation_interfaces.srv import SetMode
from geometry_msgs.msg import Twist
import random

class MockActuatorComponent(Node):
    """Example actuator component that extends the foundation."""
    
    def __init__(self):
        super().__init__('mock_actuator')
        
        # Component status
        self.status = ComponentStatus()
        self.status.component_name = 'mock_actuator'
        self.status.component_type = 'actuator'
        self.status.status = ComponentStatus.OFFLINE
        
        # Publishers
        self.status_pub = self.create_publisher(ComponentStatus, '~/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Services
        self.mode_service = self.create_service(
            SetMode, 
            '~/set_mode', 
            self.set_mode_callback
        )
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            '~/cmd_vel_input',
            self.cmd_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Internal state
        self.current_mode = 0  # AUTO
        self.enabled = False
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.heartbeat_timer = self.create_timer(0.1, self.heartbeat)
        
        # Initialize component
        self.initialize_component()
        
        self.get_logger().info('Mock Actuator Component started')
    
    def initialize_component(self):
        """Initialize the component."""
        try:
            # Simulate initialization process
            self.get_logger().info('Initializing actuator component...')
            
            # Set status to online
            self.status.status = ComponentStatus.ONLINE
            self.status.error_message = ""
            self.enabled = True
            
            self.get_logger().info('Actuator component initialized successfully')
            
        except Exception as e:
            self.status.status = ComponentStatus.ERROR
            self.status.error_message = str(e)
            self.enabled = False
            self.get_logger().error(f'Failed to initialize actuator: {e}')
    
    def set_mode_callback(self, request, response):
        """Handle mode change requests."""
        self.get_logger().info(f'Mode change requested: {request.mode}')
        
        self.current_mode = request.mode
        
        if request.mode == 0:  # AUTO
            self.enabled = True
            response.success = True
            response.message = "Switched to AUTO mode"
        elif request.mode == 1:  # MANUAL
            self.enabled = True
            response.success = True
            response.message = "Switched to MANUAL mode"
        elif request.mode == 3:  # MAINTENANCE
            self.enabled = False
            response.success = True
            response.message = "Switched to MAINTENANCE mode"
        else:
            response.success = False
            response.message = f"Unsupported mode: {request.mode}"
        
        return response
    
    def cmd_callback(self, msg):
        """Handle velocity commands."""
        if not self.enabled:
            return
        
        # Limit velocities
        limited_msg = Twist()
        limited_msg.linear.x = max(-self.max_linear_vel, 
                                  min(self.max_linear_vel, msg.linear.x))
        limited_msg.angular.z = max(-self.max_angular_vel, 
                                   min(self.max_angular_vel, msg.angular.z))
        
        # Publish the command
        self.cmd_vel_pub.publish(limited_msg)
        
        self.get_logger().debug(
            f'Publishing cmd_vel: linear={limited_msg.linear.x:.2f}, '
            f'angular={limited_msg.angular.z:.2f}'
        )
    
    def heartbeat(self):
        """Regular heartbeat to update component status."""
        # Simulate some random component behavior
        if random.random() < 0.001:  # 0.1% chance of temporary error
            self.status.status = ComponentStatus.ERROR
            self.status.error_message = "Simulated temporary error"
            self.enabled = False
        elif self.status.status == ComponentStatus.ERROR and random.random() < 0.1:
            # 10% chance to recover from error
            self.status.status = ComponentStatus.ONLINE
            self.status.error_message = ""
            self.enabled = True
        
        self.status.last_update_time = self.get_clock().now().nanoseconds / 1e9
    
    def publish_status(self):
        """Publish component status."""
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(self.status)

def main(args=None):
    rclpy.init(args=args)
    node = MockActuatorComponent()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
