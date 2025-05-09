#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations
from tier4_system_msgs.srv import ChangeOperationMode
import time
import math


class AWNavigationNode(Node):

    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our navigation is started")
        self.goal_poses = []
        self.current_goal_index = 0

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 10)

        self.odom_listener = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10)

        self.change_mode_srv = self.create_client(
            ChangeOperationMode, '/system/operation_mode/change_operation_mode')
        self.change_mode_req = ChangeOperationMode.Request()

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 3720.771728515625
        initial_pose.pose.pose.position.y = 73680.1796875
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.8532157299434914
        initial_pose.pose.pose.orientation.w = 0.5215581637526109

        time.sleep(5)
        self.initial_pose_publisher.publish(initial_pose)

        # Goal Poses (updated)
        self.goal_poses.append({
            'x': 3836.241943359375,
            'y': 73765.3515625,
            'z': -0.5219453617499283,
            'w': 0.8529789208109052
        })

        self.goal_poses.append({
            'x': 3802.692138671875,
            'y': 73765.03125,
            'z': 0.2782383876323267,
            'w': 0.9605120507561387
        })

        time.sleep(5)
        self.publish_goal()

    def send_request(self):
        self.change_mode_req.mode = 2
        future = self.change_mode_srv.call_async(self.change_mode_req)

    def odom_callback(self, msg: Odometry):
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]
        distance_to_goal = math.hypot(
            current_pose.position.x - goal_pose['x'],
            current_pose.position.y - goal_pose['y']
        )
        if distance_to_goal < 0.3:
            print(f"Reached goal {self.current_goal_index} (distance: {distance_to_goal})")
            self.publish_next_goal()

    def publish_next_goal(self):
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            self.publish_goal()
        else:
            self.get_logger().info("All goals explored!")
            self.stop()

    def publish_goal(self):
        goal = self.goal_poses[self.current_goal_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = goal['x']
        pose_msg.pose.position.y = goal['y']
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = goal['z']
        pose_msg.pose.orientation.w = goal['w']
        self.goal_pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published goal {self.current_goal_index}")
        time.sleep(5)
        self.send_request()

    def stop(self):
        self.get_logger().info("Stopping the node")
        rclpy.shutdown()
        raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = AWNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
