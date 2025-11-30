#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import os
from cv_bridge import CvBridge
import cv2
from datetime import datetime


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Parameters: save directory and cooldown
        self.declare_parameter('save_dir', os.path.expanduser('~/camera/image_raw'))
        self.declare_parameter('save_cooldown', 5.0)

        # Use sensor data QoS so it matches Gazebo camera publisher
        self.sub_image = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscribe to odometry to get robot position
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.last_image = None
        self.frame_count = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.range_threshold = 1.5  # meters - capture when within 1.5m of any obstacle
        
        # Obstacle positions (from URDF)
        self.obstacles = [
            (2.0, 0.0),    # obs1
            (3.0, 0.0),    # obs2
            (3.0, 1.0),    # obs3
            (3.0, -1.0),   # obs4
            (4.0, 0.5),    # obs5
        ]
        
        self.in_range_obstacles = set()
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Publisher to notify when an image is saved
        self.save_pub = self.create_publisher(String, '/image_save/status', 10)

        # CvBridge for conversion
        self.bridge = CvBridge()

        # Directory to save images (from parameter)
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').get_parameter_value().string_value)
        os.makedirs(self.save_dir, exist_ok=True)

        # Per-obstacle cooldown (seconds) to avoid repeated saves
        self.save_cooldown = float(self.get_parameter('save_cooldown').get_parameter_value().double_value)
        self.last_saved_time = {i: 0.0 for i in range(len(self.obstacles))}

        self.get_logger().info(
            'ImageSubscriber started, listening to /camera/image_raw'
        )
        self.get_logger().info(
            f'Range threshold set to {self.range_threshold}m for obstacle detection'
        )

    def image_callback(self, msg: Image):
        # Debug: log when image is received
        self.get_logger().info(f"Image received: frame {self.frame_count+1}, encoding={msg.encoding}, size={msg.width}x{msg.height}")
        self.last_image = msg
        self.frame_count += 1

    def odom_callback(self, msg: Odometry):
        # Update robot position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Check which obstacles are in range
        self.in_range_obstacles.clear()
        for idx, (obs_x, obs_y) in enumerate(self.obstacles):
            distance = math.sqrt((self.robot_x - obs_x)**2 + (self.robot_y - obs_y)**2)
            self.get_logger().info(f"Checking obstacle {idx}: robot ({self.robot_x:.2f},{self.robot_y:.2f}) obs ({obs_x:.2f},{obs_y:.2f}) distance={distance:.2f}")
            if distance <= self.range_threshold:
                self.in_range_obstacles.add(idx)

                # If we have a recent image, save it immediately when in range
                if self.last_image is not None:
                    # Check cooldown
                    now = datetime.utcnow().timestamp()
                    if now - self.last_saved_time.get(idx, 0.0) < self.save_cooldown:
                        # recently saved for this obstacle
                        self.get_logger().info(f"Obstacle {idx} in range but cooldown active ({now - self.last_saved_time.get(idx, 0.0):.2f}s < {self.save_cooldown}s)")
                        continue
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding='bgr8')
                    except Exception as e:
                        self.get_logger().error(f'Failed converting image: {e}')
                        return

                    timestamp = datetime.utcnow().strftime('%Y%m%d_%H%M%S_%f')
                    filename = f'obstacle_{idx}_{timestamp}.png'
                    filepath = os.path.join(self.save_dir, filename)

                    try:
                        cv2.imwrite(filepath, cv_image)
                        msg = String()
                        msg.data = f'saved:{filepath}'
                        self.save_pub.publish(msg)
                        self.get_logger().info(f'Saved image for obstacle {idx} -> {filepath}')
                        # update last saved time
                        self.last_saved_time[idx] = now
                        # Append record to log file
                        try:
                            logpath = os.path.join(self.save_dir, 'save_log.txt')
                            with open(logpath, 'a') as lf:
                                lf.write(f"{datetime.utcnow().isoformat()}\t{filepath}\n")
                        except Exception as e:
                            self.get_logger().error(f'Failed to write save log: {e}')
                    except Exception as e:
                        self.get_logger().error(f'Failed to write image to disk: {e}')

    def timer_callback(self):
        if self.last_image is None:
            self.get_logger().info(
                'No image received yet on /camera/image_raw...'
            )
            return

        w = self.last_image.width
        h = self.last_image.height
        enc = self.last_image.encoding

        # Get current distance to nearest obstacle
        min_distance = float('inf')
        nearest_obstacle_idx = -1
        for idx, (obs_x, obs_y) in enumerate(self.obstacles):
            distance = math.sqrt((self.robot_x - obs_x)**2 + (self.robot_y - obs_y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle_idx = idx

        # Check if in range
        if self.in_range_obstacles:
            self.get_logger().warn(
                f' OBSTACLE IN RANGE! Captured image #{self.frame_count}: {w}x{h}, '
                f'encoding={enc} | Robot pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                f'Nearest obstacle #{nearest_obstacle_idx}: {min_distance:.2f}m away'
            )
        else:
            self.get_logger().info(
                f'Captured image #{self.frame_count}: {w}x{h}, encoding={enc} | '
                f'Robot pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                f'Nearest obstacle: {min_distance:.2f}m away'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
