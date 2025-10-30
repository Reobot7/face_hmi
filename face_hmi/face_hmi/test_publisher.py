#!/usr/bin/env python3
"""
Test publisher for faceHMI
Randomly moves the gaze to different positions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from face_hmi_msgs.msg import Health
import random
import math


class TestPublisher(Node):
    """Test node that publishes random attention commands"""
    
    def __init__(self):
        super().__init__('face_hmi_test_publisher')
        
        # Publishers
        self.pub_attention = self.create_publisher(Vector3, '/hmi/attention', 10)
        self.pub_activity = self.create_publisher(String, '/hmi/activity', 10)
        self.pub_health = self.create_publisher(Health, '/hmi/health', 10)
        self.pub_label = self.create_publisher(String, '/hmi/attention_label', 10)
        
        # Timer for random gaze changes (every 2-5 seconds)
        self.create_timer(3.0, self.publish_random_attention)
        
        # Timer for activity changes (every 10 seconds)
        self.create_timer(10.0, self.publish_random_activity)
        
        # Timer for health updates (every 1 second)
        self.create_timer(1.0, self.publish_health)
        
        # State
        self.activities = ['idle', 'working', 'announce_move', 'moving']
        self.current_activity_index = 0
        self.battery = 100.0
        self.temp = 25.0
        
        self.get_logger().info('Test publisher started')
    
    def publish_random_attention(self):
        """Publish random attention direction"""
        msg = Vector3()
        
        # Generate random position in normalized range [-1, 1]
        msg.x = random.uniform(-0.8, 0.8)
        msg.y = random.uniform(-0.6, 0.6)
        msg.z = 0.0
        
        self.pub_attention.publish(msg)
        self.get_logger().info(f'Published attention: x={msg.x:.2f}, y={msg.y:.2f}')
        
        # Publish label
        labels = [
            'Looking around',
            'Scanning area',
            'Checking sensors',
            'Observing',
            'Monitoring'
        ]
        label_msg = String()
        label_msg.data = random.choice(labels)
        self.pub_label.publish(label_msg)
    
    def publish_random_activity(self):
        """Cycle through activity states"""
        msg = String()
        msg.data = self.activities[self.current_activity_index]
        self.pub_activity.publish(msg)
        
        self.get_logger().info(f'Published activity: {msg.data}')
        
        self.current_activity_index = (self.current_activity_index + 1) % len(self.activities)
    
    def publish_health(self):
        """Publish simulated health data"""
        msg = Health()
        
        # Simulate battery drain
        self.battery = max(0.0, self.battery - 0.05)
        if self.battery < 1.0:
            self.battery = 100.0  # Reset
        
        # Simulate temperature fluctuation
        self.temp = 25.0 + 30.0 * math.sin(self.get_clock().now().nanoseconds / 1e10)
        
        msg.battery_pct = self.battery
        msg.temp_c = self.temp
        msg.net_ok = random.random() > 0.1  # 90% network OK
        msg.charging = self.battery < 30.0  # Charging when low
        
        self.pub_health.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
