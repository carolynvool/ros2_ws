from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_system_msgs.srv import ChangeOperationMode


class CarNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("Mission planning started")
        self.goal_poses = []
        self.current_goal_index = 0

        # Publishers for initial and goal poses
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, "/planning/mission_planning/goal", 10)

        # Subscriber for vehicle position
        self.odom_listener = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.odom_callback,
            10
        )

        # Service client for changing operation mode
        self.change_mode_srv = self.create_client(ChangeOperationMode, '/system/operation_mode/change_operation_mode')
        self.change_mode_req = ChangeOperationMode.Request()

        # Initialize pose and goals
        self.current_goal_index = 0
        self.setup_initial_pose()
        self.setup_goals()
        self.publish_goal()
        self.send_request()

    def odom_callback(self, msg: Odometry):
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]
        distance_to_goal = (
            ((current_pose.position.x - goal_pose['x']) ** 2 +
             (current_pose.position.y - goal_pose['y']) ** 2) ** 0.5
        )
        if distance_to_goal < 0.3:
            self.publish_next_goal()

    def publish_next_goal(self):
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            self.publish_goal()
        else:
            self.get_logger().info("All goals reached!")
            self.stop()

    def publish_goal(self):
        goal = self.goal_poses[self.current_goal_index]
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal['x']
        goal_msg.pose.position.y = goal['y']
        goal_msg.pose.orientation.x = goal['xx']
        goal_msg.pose.orientation.y = goal['yy']
        goal_msg.pose.orientation.z = goal['zz']
        goal_msg.pose.orientation.w = goal['w']
        self.goal_pose_publisher.publish(goal_msg)

    def setup_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 3720.771728515625
        initial_pose.pose.pose.position.y = 73680.1796875
        initial_pose.pose.pose.orientation.z = 0.8532157299434914
        initial_pose.pose.pose.orientation.w = 0.5215581637526109
        self.initial_pose_publisher.publish(initial_pose)

    def setup_goals(self):
        self.goal_poses = [
            {
                'x': 3836.241943359375,
                'y': 73765.3515625,
                'xx': 0.0,
                'yy': 0.0,
                'zz': -0.5219453617499283,
                'w': 0.8529789208109052
            },
            {
                'x': 3802.692138671875,
                'y': 73765.03125,
                'xx': 0.0,
                'yy': 0.0,
                'zz': 0.2782383876323267,
                'w': 0.9605120507561387
            }
        ]

    def send_request(self):
        self.change_mode_req.mode = 2  # Enable autonomous mode
        self.change_mode_srv.call_async(self.change_mode_req)        

def main(args=None):
    rclpy.init(args=args)
    node = CarNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()