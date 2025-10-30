#!/usr/bin/env python3
"""
faceHMI ROS2 Node
Displays dual eyes on HDMI display showing attention, activity state, and health status
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PointStamped
from std_msgs.msg import String
from face_hmi_msgs.msg import Health
import pygame
import math
import sys
from typing import Optional, Tuple

try:
    from tf2_ros import Buffer, TransformListener
    from tf2_geometry_msgs import do_transform_point
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class FaceHMINode(Node):
    """Main HMI node for dual-eye face display"""
    
    def __init__(self):
        super().__init__('face_hmi')
        
        # Declare parameters
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('fov_x_deg', 90.0)
        self.declare_parameter('fov_y_deg', 60.0)
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('fps', 60)
        
        # Get parameters
        self.camera_frame = self.get_parameter('camera_frame').value
        self.fov_x = math.radians(self.get_parameter('fov_x_deg').value)
        self.fov_y = math.radians(self.get_parameter('fov_y_deg').value)
        self.fullscreen = self.get_parameter('fullscreen').value
        self.fps = self.get_parameter('fps').value
        
        # Initialize pygame
        pygame.init()
        
        # Set up display
        if self.fullscreen:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((1920, 1080))
        
        pygame.display.set_caption('faceHMI')
        self.clock = pygame.time.Clock()
        
        self.width, self.height = self.screen.get_size()
        self.get_logger().info(f'Display initialized: {self.width}x{self.height}')
        
        # State variables
        self.attention_x = 0.0  # -1 to 1
        self.attention_y = 0.0  # -1 to 1
        self.activity = 'idle'
        self.attention_label = ''
        self.battery_pct = 100.0
        self.temp_c = 25.0
        self.net_ok = True
        self.charging = False
        
        # TF2 setup
        if TF2_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warn('TF2 not available, /hmi/attention_target will be ignored')
        
        # Create subscriptions
        self.sub_attention = self.create_subscription(
            Vector3, '/hmi/attention', self.attention_callback, 10)
        
        if TF2_AVAILABLE:
            self.sub_attention_target = self.create_subscription(
                PointStamped, '/hmi/attention_target', self.attention_target_callback, 10)
        
        self.sub_attention_label = self.create_subscription(
            String, '/hmi/attention_label', self.attention_label_callback, 10)
        
        self.sub_activity = self.create_subscription(
            String, '/hmi/activity', self.activity_callback, 10)
        
        self.sub_health = self.create_subscription(
            Health, '/hmi/health', self.health_callback, 10)
        
        # Animation state
        self.flash_phase = 0.0
        
        self.get_logger().info('faceHMI node started')
    
    def attention_callback(self, msg: Vector3):
        """Direct attention control"""
        self.attention_x = max(-1.0, min(1.0, msg.x))
        self.attention_y = max(-1.0, min(1.0, msg.y))
    
    def attention_target_callback(self, msg: PointStamped):
        """Transform 3D point to screen coordinates"""
        if not TF2_AVAILABLE:
            return
        
        try:
            # Transform to camera frame
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, msg.header.frame_id, rclpy.time.Time())
            point_cam = do_transform_point(msg, transform)
            
            # Project to normalized screen coordinates
            if point_cam.point.z > 0.01:  # Avoid division by zero
                # Calculate angles
                angle_x = math.atan2(point_cam.point.x, point_cam.point.z)
                angle_y = math.atan2(point_cam.point.y, point_cam.point.z)
                
                # Normalize by FOV
                self.attention_x = max(-1.0, min(1.0, angle_x / (self.fov_x / 2)))
                self.attention_y = max(-1.0, min(1.0, angle_y / (self.fov_y / 2)))
        
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
    
    def attention_label_callback(self, msg: String):
        """Update attention label"""
        self.attention_label = msg.data
    
    def activity_callback(self, msg: String):
        """Update activity state"""
        valid_states = ['idle', 'working', 'announce_move', 'moving']
        if msg.data in valid_states:
            self.activity = msg.data
        else:
            self.get_logger().warn(f'Invalid activity state: {msg.data}')
    
    def health_callback(self, msg: Health):
        """Update health status"""
        self.battery_pct = msg.battery_pct
        self.temp_c = msg.temp_c
        self.net_ok = msg.net_ok
        self.charging = msg.charging
    
    def get_activity_color(self) -> Tuple[int, int, int]:
        """Get color based on activity state"""
        if self.activity == 'idle':
            return (240, 240, 255)  # White-ish
        elif self.activity == 'working':
            return (100, 150, 255)  # Blue
        elif self.activity == 'announce_move':
            # Flash white
            intensity = int(255 * (0.5 + 0.5 * math.sin(self.flash_phase)))
            return (intensity, intensity, intensity)
        elif self.activity == 'moving':
            return (180, 220, 255)  # Light blue
        return (255, 255, 255)
    
    def get_battery_color(self) -> Tuple[int, int, int]:
        """Get color based on battery level"""
        if self.battery_pct > 50:
            return (0, 255, 0)  # Green
        elif self.battery_pct > 20:
            return (255, 255, 0)  # Yellow
        else:
            return (255, 0, 0)  # Red
    
    def get_temp_color(self) -> Tuple[int, int, int]:
        """Get color based on temperature"""
        if self.temp_c < 60:
            return (0, 255, 0)  # Green
        elif self.temp_c < 75:
            return (255, 255, 0)  # Yellow
        else:
            return (255, 0, 0)  # Red
    
    def draw_eye(self, center_x: int, center_y: int, is_left: bool):
        """Draw a single eye with pupil and health ring"""
        eye_radius = min(self.width, self.height) // 8
        pupil_radius = eye_radius // 3
        ring_width = 8
        
        # Get colors
        activity_color = self.get_activity_color()
        
        # Draw eye background (sclera)
        pygame.draw.circle(self.screen, (255, 255, 255), (center_x, center_y), eye_radius)
        
        # Calculate pupil position based on attention
        max_offset = eye_radius - pupil_radius - 10
        pupil_x = int(center_x + self.attention_x * max_offset)
        pupil_y = int(center_y - self.attention_y * max_offset)  # Invert Y for screen coords
        
        # Draw pupil
        pygame.draw.circle(self.screen, (0, 0, 0), (pupil_x, pupil_y), pupil_radius)
        
        # Draw iris with activity color
        iris_radius = pupil_radius + 15
        pygame.draw.circle(self.screen, activity_color, (pupil_x, pupil_y), iris_radius, 3)
        
        # Draw health ring
        if is_left:
            # Left eye: battery
            color = self.get_battery_color()
            arc_angle = (self.battery_pct / 100.0) * 2 * math.pi
        else:
            # Right eye: temperature
            color = self.get_temp_color()
            # Map temperature to arc (0-100°C range)
            temp_normalized = min(100.0, max(0.0, self.temp_c)) / 100.0
            arc_angle = temp_normalized * 2 * math.pi
        
        # Draw arc as segments
        if arc_angle > 0:
            segments = 60
            for i in range(int(segments * arc_angle / (2 * math.pi))):
                angle1 = -math.pi / 2 + (i / segments) * 2 * math.pi
                angle2 = -math.pi / 2 + ((i + 1) / segments) * 2 * math.pi
                
                x1 = center_x + int((eye_radius + 5) * math.cos(angle1))
                y1 = center_y + int((eye_radius + 5) * math.sin(angle1))
                x2 = center_x + int((eye_radius + 5) * math.cos(angle2))
                y2 = center_y + int((eye_radius + 5) * math.sin(angle2))
                
                pygame.draw.line(self.screen, color, (x1, y1), (x2, y2), ring_width)
        
        # Draw status icons
        icon_y = center_y + eye_radius + 30
        if is_left and self.charging:
            # Charging icon (lightning bolt)
            self.draw_text('⚡', center_x, icon_y, (255, 200, 0), size=24)
        elif not is_left and not self.net_ok:
            # Network warning icon
            self.draw_text('⚠', center_x, icon_y, (255, 100, 0), size=24)
    
    def draw_text(self, text: str, x: int, y: int, color: Tuple[int, int, int], size: int = 20):
        """Draw text centered at position"""
        font = pygame.font.Font(None, size)
        text_surface = font.render(text, True, color)
        text_rect = text_surface.get_rect(center=(x, y))
        self.screen.blit(text_surface, text_rect)
    
    def draw_hud(self):
        """Draw HUD information"""
        hud_y = 30
        hud_color = (200, 200, 200)
        
        # Activity state
        self.draw_text(f'Activity: {self.activity}', self.width // 2, hud_y, hud_color, 24)
        
        # Battery and temperature
        hud_y += 30
        self.draw_text(f'Battery: {self.battery_pct:.1f}%  Temp: {self.temp_c:.1f}°C', 
                      self.width // 2, hud_y, hud_color, 20)
        
        # Network status
        if not self.net_ok:
            hud_y += 30
            self.draw_text('Network Disconnected', self.width // 2, hud_y, (255, 100, 0), 20)
        
        # Attention label
        if self.attention_label:
            label_y = self.height - 50
            self.draw_text(self.attention_label, self.width // 2, label_y, (255, 255, 255), 28)
    
    def render(self):
        """Main render loop"""
        # Clear screen (black background)
        self.screen.fill((0, 0, 0))
        
        # Calculate eye positions
        eye_spacing = self.width // 4
        eye_y = self.height // 2
        left_eye_x = self.width // 2 - eye_spacing // 2
        right_eye_x = self.width // 2 + eye_spacing // 2
        
        # Draw eyes
        self.draw_eye(left_eye_x, eye_y, is_left=True)
        self.draw_eye(right_eye_x, eye_y, is_left=False)
        
        # Draw HUD
        self.draw_hud()
        
        # Update display
        pygame.display.flip()
        
        # Update animation phase
        self.flash_phase += 0.2
    
    def run(self):
        """Main loop"""
        running = True
        
        while running and rclpy.ok():
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        running = False
            
            # Process ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Render
            self.render()
            
            # Control frame rate
            self.clock.tick(self.fps)
        
        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FaceHMINode()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
