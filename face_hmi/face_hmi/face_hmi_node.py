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
        
        # Declare camera and display parameters
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('fov_x_deg', 90.0)
        self.declare_parameter('fov_y_deg', 60.0)
        self.declare_parameter('fullscreen', True)
        self.declare_parameter('fps', 60)
        
        # Declare eye appearance parameters
        self.declare_parameter('eye.width_divisor', 4)
        self.declare_parameter('eye.aspect_ratio', 1.2)
        self.declare_parameter('eye.spacing_divisor', 3)
        self.declare_parameter('eye.vertical_position', 0.5)
        
        self.declare_parameter('pupil.size_divisor', 2.0)
        self.declare_parameter('pupil.aspect_ratio', 1.3)
        self.declare_parameter('pupil.margin_x', 10)
        self.declare_parameter('pupil.margin_y', 10)
        
        self.declare_parameter('highlight.size_divisor', 5)
        self.declare_parameter('highlight.offset_x_divisor', 6)
        self.declare_parameter('highlight.offset_y_divisor', 6)
        
        self.declare_parameter('gradient.layers', 5)
        self.declare_parameter('gradient.outer_color', 40)
        self.declare_parameter('gradient.inner_color', 0)
        
        # Declare motion parameters
        self.declare_parameter('motion.smoothing_factor', 0.15)
        
        # Declare blink parameters
        self.declare_parameter('blink.enabled', False)
        self.declare_parameter('blink.interval_min', 3.0)
        self.declare_parameter('blink.interval_max', 8.0)
        self.declare_parameter('blink.duration', 0.15)
        
        # Declare saccade parameters
        self.declare_parameter('saccade.enabled', False)
        self.declare_parameter('saccade.speed_multiplier', 3.0)
        
        # Get parameters
        self.camera_frame = self.get_parameter('camera_frame').value
        self.fov_x = math.radians(self.get_parameter('fov_x_deg').value)
        self.fov_y = math.radians(self.get_parameter('fov_y_deg').value)
        self.fullscreen = self.get_parameter('fullscreen').value
        self.fps = self.get_parameter('fps').value
        
        # Get appearance parameters
        self.eye_width_divisor = self.get_parameter('eye.width_divisor').value
        self.eye_aspect_ratio = self.get_parameter('eye.aspect_ratio').value
        self.eye_spacing_divisor = self.get_parameter('eye.spacing_divisor').value
        self.eye_vertical_position = self.get_parameter('eye.vertical_position').value
        
        self.pupil_size_divisor = self.get_parameter('pupil.size_divisor').value
        self.pupil_aspect_ratio = self.get_parameter('pupil.aspect_ratio').value
        self.pupil_margin_x = self.get_parameter('pupil.margin_x').value
        self.pupil_margin_y = self.get_parameter('pupil.margin_y').value
        
        self.highlight_size_divisor = self.get_parameter('highlight.size_divisor').value
        self.highlight_offset_x_divisor = self.get_parameter('highlight.offset_x_divisor').value
        self.highlight_offset_y_divisor = self.get_parameter('highlight.offset_y_divisor').value
        
        self.gradient_layers = self.get_parameter('gradient.layers').value
        self.gradient_outer_color = self.get_parameter('gradient.outer_color').value
        self.gradient_inner_color = self.get_parameter('gradient.inner_color').value
        
        # Get motion parameters
        self.smoothing_factor = self.get_parameter('motion.smoothing_factor').value
        
        # Get blink parameters
        self.blink_enabled = self.get_parameter('blink.enabled').value
        self.blink_interval_min = self.get_parameter('blink.interval_min').value
        self.blink_interval_max = self.get_parameter('blink.interval_max').value
        self.blink_duration = self.get_parameter('blink.duration').value
        
        # Get saccade parameters
        self.saccade_enabled = self.get_parameter('saccade.enabled').value
        self.saccade_speed_multiplier = self.get_parameter('saccade.speed_multiplier').value
        
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
        self.target_attention_x = 0.0  # Target position -1 to 1
        self.target_attention_y = 0.0  # Target position -1 to 1
        self.current_attention_x = 0.0  # Current interpolated position
        self.current_attention_y = 0.0  # Current interpolated position
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
        
        # Blink state
        self.blink_phase = 0.0  # 0.0 = fully open, 1.0 = fully closed
        self.is_blinking = False
        self.blink_timer = 0.0
        self.next_blink_time = 0.0
        if self.blink_enabled:
            import random
            self.next_blink_time = random.uniform(self.blink_interval_min, self.blink_interval_max)
        
        self.get_logger().info('faceHMI node started')
        self.get_logger().info(f'Eye appearance: width_div={self.eye_width_divisor}, aspect={self.eye_aspect_ratio}')
        self.get_logger().info(f'Motion: smoothing={self.smoothing_factor}')
        if self.blink_enabled:
            self.get_logger().info(f'Blink: enabled, interval={self.blink_interval_min}-{self.blink_interval_max}s, duration={self.blink_duration}s')
    
    def attention_callback(self, msg: Vector3):
        """Direct attention control"""
        self.target_attention_x = max(-1.0, min(1.0, msg.x))
        self.target_attention_y = max(-1.0, min(1.0, msg.y))
    
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
                self.target_attention_x = max(-1.0, min(1.0, angle_x / (self.fov_x / 2)))
                self.target_attention_y = max(-1.0, min(1.0, angle_y / (self.fov_y / 2)))
        
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
    
    def update_blink(self, dt: float):
        """Update blink animation"""
        if not self.blink_enabled:
            return
        
        import random
        
        self.blink_timer += dt
        
        if self.is_blinking:
            # Currently blinking
            blink_progress = self.blink_timer / self.blink_duration
            
            if blink_progress < 0.5:
                # Closing (0.0 -> 1.0)
                self.blink_phase = blink_progress * 2.0
            else:
                # Opening (1.0 -> 0.0)
                self.blink_phase = 2.0 - (blink_progress * 2.0)
            
            if blink_progress >= 1.0:
                # Blink complete
                self.is_blinking = False
                self.blink_phase = 0.0
                self.blink_timer = 0.0
                self.next_blink_time = random.uniform(self.blink_interval_min, self.blink_interval_max)
        else:
            # Waiting for next blink
            if self.blink_timer >= self.next_blink_time:
                self.is_blinking = True
                self.blink_timer = 0.0
    
    def draw_eye(self, center_x: int, center_y: int, is_left: bool):
        """Draw a single eye with moving pupil"""
        # Eye dimensions (oval shape) - from config
        eye_width = min(self.width, self.height) // self.eye_width_divisor
        eye_height = int(eye_width * self.eye_aspect_ratio)
        
        # Apply blink effect (reduce visible height)
        visible_eye_height = int(eye_height * (1.0 - self.blink_phase))
        if visible_eye_height < 2:
            visible_eye_height = 2  # Minimum height when blinking
        
        # Pupil dimensions - from config
        pupil_width = int(eye_width / self.pupil_size_divisor)
        pupil_height = int(pupil_width * self.pupil_aspect_ratio)
        
        # Highlight dimensions - from config
        highlight_radius = int(pupil_width / self.highlight_size_divisor)
        
        # Draw white of the eye (sclera) - oval with blink effect
        eye_rect = pygame.Rect(
            center_x - eye_width // 2,
            center_y - visible_eye_height // 2,
            eye_width,
            visible_eye_height
        )
        pygame.draw.ellipse(self.screen, (255, 255, 255), eye_rect)
        
        # If mostly closed, don't draw pupil
        if self.blink_phase > 0.9:
            return
        
        # Calculate pupil position based on attention
        # Limit movement to stay within the white part - from config
        max_offset_x = (eye_width - pupil_width) // 2 - self.pupil_margin_x
        max_offset_y = (visible_eye_height - pupil_height) // 2 - self.pupil_margin_y
        
        # Ensure max_offset is positive
        if max_offset_y < 0:
            max_offset_y = 0
        
        pupil_x = int(center_x + self.current_attention_x * max_offset_x)
        pupil_y = int(center_y - self.current_attention_y * max_offset_y)  # Invert Y for screen coords
        
        # Draw pupil with gradient effect - from config
        for i in range(self.gradient_layers):
            scale = 1.0 - (i * (1.0 / self.gradient_layers))
            # Interpolate color from outer to inner
            color_value = int(self.gradient_outer_color - 
                            (self.gradient_outer_color - self.gradient_inner_color) * 
                            (i / self.gradient_layers))
            
            gradient_rect = pygame.Rect(
                pupil_x - int(pupil_width * scale) // 2,
                pupil_y - int(pupil_height * scale) // 2,
                int(pupil_width * scale),
                int(pupil_height * scale)
            )
            pygame.draw.ellipse(self.screen, (color_value, color_value, color_value), gradient_rect)
        
        # Draw highlight (white spot) - from config
        highlight_offset_x = -int(pupil_width / self.highlight_offset_x_divisor)
        highlight_offset_y = -int(pupil_height / self.highlight_offset_y_divisor)
        highlight_pos = (
            pupil_x + highlight_offset_x,
            pupil_y + highlight_offset_y
        )
        pygame.draw.circle(self.screen, (255, 255, 255), highlight_pos, highlight_radius)
    
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
        battery_color = self.get_battery_color()
        temp_color = self.get_temp_color()
        
        battery_text = f'Battery: {self.battery_pct:.1f}%'
        temp_text = f'Temp: {self.temp_c:.1f}°C'
        
        # Draw battery info
        self.draw_text(battery_text, self.width // 2 - 150, hud_y, battery_color, 20)
        
        # Draw charging icon if charging
        if self.charging:
            self.draw_text('⚡', self.width // 2 - 50, hud_y, (255, 200, 0), 24)
        
        # Draw temperature info
        self.draw_text(temp_text, self.width // 2 + 100, hud_y, temp_color, 20)
        
        # Network status
        if not self.net_ok:
            hud_y += 30
            self.draw_text('⚠ Network Disconnected', self.width // 2, hud_y, (255, 100, 0), 20)
        
        # Attention label
        if self.attention_label:
            label_y = self.height - 50
            self.draw_text(self.attention_label, self.width // 2, label_y, (255, 255, 255), 28)
    
    def update_attention_interpolation(self):
        """Smoothly interpolate attention position"""
        # Linear interpolation (lerp) - smoothing factor from config
        self.current_attention_x += (self.target_attention_x - self.current_attention_x) * self.smoothing_factor
        self.current_attention_y += (self.target_attention_y - self.current_attention_y) * self.smoothing_factor
    
    def render(self, dt: float):
        """Main render loop"""
        # Update smooth interpolation
        self.update_attention_interpolation()
        
        # Update blink animation
        self.update_blink(dt)
        
        # Clear screen (black background)
        self.screen.fill((0, 0, 0))
        
        # Calculate eye positions - from config
        eye_spacing = self.width // self.eye_spacing_divisor
        eye_y = int(self.height * self.eye_vertical_position)
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
            
            # Get delta time
            dt = self.clock.tick(self.fps) / 1000.0  # Convert to seconds
            
            # Render
            self.render(dt)
        
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
