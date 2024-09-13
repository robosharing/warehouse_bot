import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster
import math
import time
from robot_interfaces.msg import RobotState  # Replace with your actual package name
from geometry_msgs.msg import TransformStamped
class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')

        self.declare_parameter('robot_name', 'AGV1')
        self.declare_parameter('initial_x', 166.0)
        self.declare_parameter('initial_y', -44.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value

        self.current_position = PoseStamped().pose.position
        self.current_position.x = self.initial_x
        self.current_position.y = self.initial_y
        self.current_position.z = 0.0

        self.current_orientation = self.get_quaternion_from_yaw(self.initial_yaw)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.target_position = None

        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def set_goal(self, pose):
        self.target_position = pose.pose.position
        self.get_logger().info(f'Received new target position: x={self.target_position.x}, y={self.target_position.y}')

    def control_loop(self):
        if self.target_position is None:
            self.publish_cmd_vel(0.0, 0.0)
            return

        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        distance = math.sqrt(dx**2 + dy**2)

        target_yaw = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_quaternion(self.current_orientation)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)

        linear_speed = min(0.5, distance)
        angular_speed = min(0.5, yaw_error)

        if abs(yaw_error) > 0.1:
            linear_speed = 0.0

        self.publish_cmd_vel(linear_speed, angular_speed)

        if distance < 0.05:
            self.get_logger().info('Target position reached.')
            self.target_position = None
            self.publish_cmd_vel(0.0, 0.0)

    def update_position(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist)

        # Update the position based on linear and angular speeds
        dt = 0.1  # Time step

        # Update the yaw
        current_yaw = self.get_yaw_from_quaternion(self.current_orientation)
        new_yaw = current_yaw + angular_speed * dt

        # Update the position
        self.current_position.x += linear_speed * math.cos(new_yaw) * dt
        self.current_position.y += linear_speed * math.sin(new_yaw) * dt

        # Create a new transform for the updated position
        new_transform = TransformStamped()
        new_transform.header.stamp = self.get_clock().now().to_msg()
        new_transform.header.frame_id = 'map'
        new_transform.child_frame_id = f'{self.robot_name}.base_footprint'
        new_transform.transform.translation.x = float(self.current_position.x)
        new_transform.transform.translation.y = float(self.current_position.y)
        new_transform.transform.translation.z = float(self.current_position.z)

        # Update the orientation
        new_transform.transform.rotation = self.get_quaternion_from_yaw(new_yaw)

        # Broadcast the new transform
        self.tf_broadcaster.sendTransform(new_transform)

        # Update the current orientation
        self.current_orientation = new_transform.transform.rotation

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.w = float(math.cos(yaw / 2))
        q.z = float(math.sin(yaw / 2))
        q.x = 0.0
        q.y = 0.0
        return q

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class NavigateToPoseServer(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback
        )
        self.get_logger().info('Navigate to Pose Action Server is up and running.')

        # Initialize the SimplePathPlanner
        self.path_planner = SimplePathPlanner()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.path_planner.set_goal(goal_handle.request.pose)

        while rclpy.ok() and self.path_planner.target_position is not None:
            rclpy.spin_once(self.path_planner, timeout_sec=0.1)
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose.pose.position = self.path_planner.current_position
            feedback_msg.current_pose.pose.orientation = self.path_planner.current_orientation
            feedback_msg.navigation_time = Duration()
            feedback_msg.estimated_time_remaining = Duration()
            feedback_msg.number_of_recoveries = 0

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        if self.path_planner.target_position is None:
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
            result = NavigateToPose.Result()
            return result

        goal_handle.abort()
        self.get_logger().info('Goal aborted')
        result = NavigateToPose.Result()
        result.error_code = NavigateToPose.Result.ABORT
        return result

def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_server = NavigateToPoseServer()

    try:
        rclpy.spin(navigate_to_pose_server)
    except KeyboardInterrupt:
        pass
    finally:
        navigate_to_pose_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
