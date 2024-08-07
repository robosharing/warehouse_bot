# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

import requests
from urllib.error import HTTPError
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import threading

class RobotAPI(Node):
    # The constructor below accepts parameters typically required to submit
    # HTTP requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        super().__init__('robot_api_node')
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False

        self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.thread.start()

    def __del__(self):
        rclpy.shutdown()
        self.thread.join()

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        if self.data() is None:
            return False
        return True

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        response = self.data(robot_name)
        if response is not None:
            if self.debug:
                print(f'Response: {response}')
            if not response['success']:
                print(f'Response for {robot_name} was not successful')
                return None

            position = response['data']['position']
            x = position['x']
            y = position['y']
            angle = position['yaw']
            return [x, y, angle]

        print(f'No response received for {robot_name}')
        return None

    def navigate(self, robot_name: str, cmd_id: int, pose, map_name: str, speed_limit=0.0):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        assert(len(pose) > 2)
        self.navigate_to_pose(pose)
        # Check for the result or feedback to return True or False
        return True  # Modify based on your success criteria

    def start_process(self, robot_name: str, cmd_id: int, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        url = self.prefix + f"/open-rmf/rmf_demos_fm/start_task?robot_name={robot_name}&cmd_id={cmd_id}"
        # data fields: task, map_name, destination{}, data{}
        data = {'task': process, 'map_name': map_name}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False

    def stop(self, robot_name: str, cmd_id: int):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        url = self.prefix + f'/open-rmf/rmf_demos_fm/stop_robot?robot_name={robot_name}&cmd_id={cmd_id}'
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False

    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        response = self.data(robot_name)
        if response is None:
            return None

        if response is not None:
            arrival = response['data'].get('destination_arrival')
            if arrival is not None:
                if arrival.get('cmd_id') != cmd_id:
                    return None
                return arrival.get('duration')
            else:
                return None

        else:
            return None

    def navigation_completed(self, robot_name: str, cmd_id: int):
        ''' Return True if the last request the robot successfully completed
            matches cmd_id. Else False.'''
        response = self.data(robot_name)
        if response is not None:
            data = response.get('data')
            if data is not None:
                return data['last_completed_request'] == cmd_id

        return False

    def process_completed(self, robot_name: str, cmd_id: int):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        return self.navigation_completed(robot_name, cmd_id)

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['battery']/100.0
        else:
            return None

    def requires_replan(self, robot_name: str):
        '''Return whether the robot needs RMF to replan'''
        response = self.data(robot_name)
        if response is not None:
            return response['data'].get('replan', False)
        return False

    def toggle_action(self, robot_name: str, toggle: bool):
        '''Request to toggle the robot's mode_teleop parameter.
           Return True if the toggle request is successful'''
        url = self.prefix + f"/open-rmf/rmf_demos_fm/toggle_action?robot_name={robot_name}"
        data = {'toggle': toggle}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False

    def data(self, robot_name=None):
        if robot_name is None:
            url = self.prefix + f'/open-rmf/rmf_demos_fm/status/'
        else:
            url = self.prefix + f'/open-rmf/rmf_demos_fm/status?robot_name={robot_name}'
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return None

    def navigate_to_pose(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]
        goal_msg.pose.pose.orientation.z = pose[2]  # You may need to convert angle to quaternion

        if not self._navigate_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return False

        self._send_goal_future = self._navigate_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        # Handle the navigation result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current location: {feedback.current_pose.pose.position}')

def main(args=None):
    rclpy.init(args=args)
    robot_api = RobotAPI('http://robot_api_prefix', 'user', 'password')
    rclpy.spin(robot_api)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
