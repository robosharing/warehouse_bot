import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from robot_interfaces.msg import RobotState  # Replace with your actual package name

class InitialPublisher(Node):
    def __init__(self):
        super().__init__('initial_publisher')

        self.declare_parameter('robot_name', 'AGV1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Инициализация текущей позиции и ориентации пустыми значениями
        self.current_position = None
        self.current_orientation = None

        # Publisher for robot state
        self.state_publisher = self.create_publisher(RobotState, 'robot_state', 10)

        # Subscriber to AMCL pose
        self.amcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # Create a future to manage shutdown
        self.future = rclpy.task.Future()

    def amcl_pose_callback(self, msg):
        if self.current_position is None and self.current_orientation is None:
            self.current_position = msg.pose.pose.position
            self.current_orientation = msg.pose.pose.orientation
            self.get_logger().info('Received initial position and orientation from AMCL')
            self.publish_initial_state()
        else:
            self.current_position = msg.pose.pose.position
            self.current_orientation = msg.pose.pose.orientation
            self.get_logger().info('Updated position and orientation from AMCL')
            self.publish_initial_state()

    def publish_initial_state(self):
        state_msg = RobotState()
        state_msg.status = 'idle'
        
        # Create and populate PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = self.current_position
        pose.pose.orientation = self.current_orientation

        state_msg.current_pose = pose
        
        # Log message details
        self.get_logger().info(f'Publishing state: status={state_msg.status}')
        self.get_logger().info(f'Pose position: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}')
        self.get_logger().info(f'Pose orientation: x={pose.pose.orientation.x}, y={pose.pose.orientation.y}, z={pose.pose.orientation.z}, w={pose.pose.orientation.w}')

        self.state_publisher.publish(state_msg)
        self.get_logger().info('State published')
        
    def set_future_result(self):
        self.get_logger().info('Setting future result to shut down the node')
        self.future.set_result(True)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPublisher()

    # Spin the node until the future is complete
    while rclpy.ok() and not node.future.done():
        rclpy.spin_once(node)

    # Destroy the node explicitly after publishing
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
