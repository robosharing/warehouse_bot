#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('cancel_goal_node')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    client.wait_for_server()

    goal_handle = client._goal_handle  # получаем текущий goal_handle

    if goal_handle:
        cancel_future = client._cancel_goal_async(goal_handle)
        rclpy.spin_until_future_complete(node, cancel_future)
        if cancel_future.result().return_code == 0:
            node.get_logger().info('Goal successfully cancelled')
        else:
            node.get_logger().error('Failed to cancel the goal')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
