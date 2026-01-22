#!/usr/bin/env python3
"""
Wheel-based odometry node for 6-wheel swerve drive robot.

Computes robot velocity from wheel encoder data using least squares,
then integrates to estimate pose. This mimics real robot behavior.

"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

import math


class WheelOdometryNode(Node):
    """
    Computes odometry from swerve drive wheel states using forward kinematics.
    """

    def __init__(self):
        super().__init__('wheel_odometry_node')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.0825)
        self.declare_parameter('wheelbase', 0.54)
        self.declare_parameter('track_width', 0.53)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', '/odom')
        
        # Joint name prefixes (order: FL, FR, ML, MR, RL, RR)
        self.declare_parameter('steer_joints', [
            'fl_steer_joint', 'fr_steer_joint',
            'ml_steer_joint', 'mr_steer_joint',
            'rl_steer_joint', 'rr_steer_joint'
        ])
        self.declare_parameter('drive_joints', [
            'fl_drive_joint', 'fr_drive_joint',
            'ml_drive_joint', 'mr_drive_joint',
            'rl_drive_joint', 'rr_drive_joint'
        ])

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.steer_joint_names = self.get_parameter('steer_joints').value
        self.drive_joint_names = self.get_parameter('drive_joints').value

        # Calculate wheel positions in robot frame
        # Front, Middle, Rear along X-axis
        x_f = self.wheelbase / 2.0
        x_m = 0.0
        x_r = -self.wheelbase / 2.0
        # Left, Right along Y-axis
        y_l = self.track_width / 2.0
        y_r = -self.track_width / 2.0

        # Wheel positions: [x, y] for each wheel
        # Order: FL, FR, ML, MR, RL, RR
        self.wheel_positions = np.array([
            [x_f, y_l],  # Front Left
            [x_f, y_r],  # Front Right
            [x_m, y_l],  # Middle Left
            [x_m, y_r],  # Middle Right
            [x_r, y_l],  # Rear Left
            [x_r, y_r]   # Rear Right
        ])

        # Pre-compute A and its pseudo-inverse for least squares
        A = []
        for (wx, wy) in self.wheel_positions:
            A.append([1, 0, -wy]) # X component equation
            A.append([0, 1,  wx]) # Y component equation
        A = np.array(A) # Shape (12, 3)
        self.P_inv = np.linalg.pinv(A)

        # Odometry state
        self.x = 0.0          # Position in odom frame (meters)
        self.y = 0.0
        self.theta = 0.0      # Heading (radians)
        self.vx = 0.0         # Velocity in base frame (m/s)
        self.vy = 0.0
        self.vtheta = 0.0     # Angular velocity (rad/s)

        self.last_time = None

        # Create subscriber to joint_states
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )

        # Create publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'Wheel Odometry Node initialized')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'  Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'  Track width: {self.track_width} m')
        self.get_logger().info(f'  Publishing TF: {self.publish_tf}')

    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint_states messages.
        Extracts wheel states and computes odometry.
        """
        try:
            # Extract steering angles and drive velocities
            steer_angles = []
            drive_velocities = []

            for steer_name, drive_name in zip(self.steer_joint_names, self.drive_joint_names):
                # Find steering joint
                if steer_name in msg.name:
                    idx = msg.name.index(steer_name)
                    steer_angles.append(msg.position[idx])
                else:
                    self.get_logger().warn(f'Steering joint {steer_name} not found in joint_states', 
                                          throttle_duration_sec=5.0)
                    return

                # Find drive joint
                if drive_name in msg.name:
                    idx = msg.name.index(drive_name)
                    drive_velocities.append(msg.velocity[idx])
                else:
                    self.get_logger().warn(f'Drive joint {drive_name} not found in joint_states',
                                          throttle_duration_sec=5.0)
                    return

            # Convert to numpy arrays
            steer_angles = np.array(steer_angles)
            drive_velocities = np.array(drive_velocities)

            # Get timestamp
            current_time = self.get_clock().now()

            # Compute robot velocity from wheel states
            self.compute_velocity(steer_angles, drive_velocities)

            # Integrate velocity to get pose
            if self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                self.integrate_odometry(dt)

            self.last_time = current_time

            # Publish odometry
            self.publish_odometry(current_time)

        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {str(e)}')

    def compute_velocity(self, steer_angles: np.ndarray, drive_velocities: np.ndarray):
        """
        Compute robot velocity (vx, vy, vtheta) from wheel states using least squares.

        For each wheel i at position (xi, yi):
        The wheel velocity in robot frame must equal:
            v_wheel_x = vx - vtheta * yi
            v_wheel_y = vy + vtheta * xi

        Where wheel velocity is:
            v_wheel_x = v_i * cos(theta_i)
            v_wheel_y = v_i * sin(theta_i)

        This gives us a linear system: A * [vx, vy, vtheta]^T = b
        We solve using least squares: (A^T * A)^-1 * A^T * b
        """
        num_wheels = 6

        # Build the b vector for least squares
        # Each wheel contributes 2 equations (x and y components)
        
        b = np.zeros(2 * num_wheels)

        for i in range(num_wheels):
            theta_i = steer_angles[i]
            v_i = drive_velocities[i] * self.wheel_radius  # Convert angular to linear velocity

            # Wheel velocity components in robot frame
            v_wheel_x = v_i * np.cos(theta_i)
            v_wheel_y = v_i * np.sin(theta_i)

            # X component equation: v_wheel_x = vx - vtheta * yi
            b[2*i] = v_wheel_x

            # Y component equation: v_wheel_y = vy + vtheta * xi
            b[2*i+1] = v_wheel_y

        # Solve least squares
        velocity = self.P_inv @ b
        self.vx = velocity[0]
        self.vy = velocity[1]
        self.vtheta = velocity[2]

       
    def integrate_odometry(self, dt: float):
        """
        Integrate velocity to update pose.
        Transforms robot-frame velocities to world frame using current heading.
        """
        if dt <= 0.0 or dt > 1.0:  # Sanity check
            return

        # Transform velocities from robot frame to world frame
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)

        vx_world = self.vx * cos_theta - self.vy * sin_theta
        vy_world = self.vx * sin_theta + self.vy * cos_theta

        # Update pose
        self.x = self.x + vx_world * dt
        self.y = self.y + vy_world * dt
        self.theta = self.theta + self.vtheta * dt

        # Normalize theta to [-pi, pi]
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def publish_odometry(self, current_time):
        """
        Publish odometry message and TF transform.
        """
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation (convert theta to quaternion)
        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = quat

        # Set velocity (in robot frame)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.vtheta

        # Set covariance (placeholder - tune these based on your robot)
        # Diagonal covariance: [x, y, z, rot_x, rot_y, rot_z]
        odom_msg.pose.covariance[0] = 0.001   # x
        odom_msg.pose.covariance[7] = 0.001   # y
        odom_msg.pose.covariance[14] = 1e6    # z (not used)
        odom_msg.pose.covariance[21] = 1e6    # rot_x (not used)
        odom_msg.pose.covariance[28] = 1e6    # rot_y (not used)
        odom_msg.pose.covariance[35] = 0.01   # rot_z

        odom_msg.twist.covariance[0] = 0.001   # vx
        odom_msg.twist.covariance[7] = 0.001   # vy
        odom_msg.twist.covariance[14] = 1e6    # vz (not used)
        odom_msg.twist.covariance[21] = 1e6    # rot_x (not used)
        odom_msg.twist.covariance[28] = 1e6    # rot_y (not used)
        odom_msg.twist.covariance[35] = 0.01   # rot_z

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Publish TF transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat

            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Convert Euler angles to quaternion.
        """
        quat = Quaternion()
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quat.w = cr * cp * cy + sr * sp * sy
        quat.x = sr * cp * cy - cr * sp * sy
        quat.y = cr * sp * cy + sr * cp * sy
        quat.z = cr * cp * sy - sr * sp * cy

        return quat


def main(args=None):
    rclpy.init(args=args)
    
    node = WheelOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()