#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foundation_interfaces.msg import RobotStatus, SystemHealth, ComponentStatus
from std_msgs.msg import Header
import psutil
import time

class SystemMonitorNode(Node):
    """System monitoring node that publishes robot status and system health."""
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Publishers
        self.robot_status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.system_health_pub = self.create_publisher(SystemHealth, 'system_health', 10)
        
        # Subscribers for component statuses
        self.component_status_sub = self.create_subscription(
            ComponentStatus, 
            '/+/status', 
            self.component_status_callback, 
            10
        )
        
        # Parameters
        self.declare_parameter('robot_id', 'foundation_robot')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('battery_topic', '')
        
        self.robot_id = self.get_parameter('robot_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Internal state
        self.component_statuses = {}
        self.robot_status = RobotStatus()
        self.robot_status.robot_id = self.robot_id
        self.robot_status.status = RobotStatus.INITIALIZING
        
        # Timers
        self.status_timer = self.create_timer(1.0 / publish_rate, self.publish_status)
        
        self.get_logger().info(f'System monitor started for robot: {self.robot_id}')
    
    def component_status_callback(self, msg):
        """Handle incoming component status messages."""
        self.component_statuses[msg.component_name] = msg
        self.get_logger().debug(f'Received status from {msg.component_name}: {msg.status}')
    
    def get_system_metrics(self):
        """Get system performance metrics."""
        try:
            cpu_usage = psutil.cpu_percent(interval=None)
            memory = psutil.virtual_memory()
            memory_usage = memory.percent
            
            # Mock battery level (replace with actual battery monitoring)
            battery_level = 85.0
            
            return cpu_usage, memory_usage, battery_level
        except Exception as e:
            self.get_logger().warn(f'Failed to get system metrics: {e}')
            return 0.0, 0.0, 0.0
    
    def determine_robot_status(self):
        """Determine overall robot status based on component statuses."""
        if not self.component_statuses:
            return RobotStatus.INITIALIZING
        
        # Check for any errors
        for status in self.component_statuses.values():
            if status.status == ComponentStatus.ERROR:
                return RobotStatus.ERROR
        
        # Check if all components are online
        all_online = all(status.status == ComponentStatus.ONLINE 
                        for status in self.component_statuses.values())
        
        if all_online:
            return RobotStatus.READY
        else:
            return RobotStatus.INITIALIZING
    
    def calculate_health_score(self, cpu_usage, memory_usage):
        """Calculate overall system health score."""
        # Simple health calculation (0-100)
        cpu_score = max(0, 100 - cpu_usage)
        memory_score = max(0, 100 - memory_usage)
        component_score = 100 if self.component_statuses else 0
        
        # Check component health
        if self.component_statuses:
            error_count = sum(1 for status in self.component_statuses.values() 
                            if status.status == ComponentStatus.ERROR)
            component_score = max(0, 100 - (error_count * 25))
        
        return (cpu_score + memory_score + component_score) / 3.0
    
    def publish_status(self):
        """Publish robot status and system health."""
        # Get system metrics
        cpu_usage, memory_usage, battery_level = self.get_system_metrics()
        
        # Update robot status
        self.robot_status.header = Header()
        self.robot_status.header.stamp = self.get_clock().now().to_msg()
        self.robot_status.status = self.determine_robot_status()
        self.robot_status.active_components = list(self.component_statuses.keys())
        self.robot_status.battery_level = battery_level
        self.robot_status.cpu_usage = cpu_usage
        self.robot_status.memory_usage = memory_usage
        
        # Publish robot status
        self.robot_status_pub.publish(self.robot_status)
        
        # Create and publish system health
        system_health = SystemHealth()
        system_health.header = Header()
        system_health.header.stamp = self.get_clock().now().to_msg()
        system_health.overall_health_score = self.calculate_health_score(cpu_usage, memory_usage)
        system_health.component_statuses = list(self.component_statuses.values())
        
        # Add warnings and errors
        system_health.warnings = []
        system_health.errors = []
        
        if cpu_usage > 80:
            system_health.warnings.append(f'High CPU usage: {cpu_usage:.1f}%')
        if memory_usage > 80:
            system_health.warnings.append(f'High memory usage: {memory_usage:.1f}%')
        if battery_level < 20:
            system_health.warnings.append(f'Low battery: {battery_level:.1f}%')
        
        for component_name, status in self.component_statuses.items():
            if status.status == ComponentStatus.ERROR:
                system_health.errors.append(f'{component_name}: {status.error_message}')
        
        self.system_health_pub.publish(system_health)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
