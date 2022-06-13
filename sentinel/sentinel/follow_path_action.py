
import rclpy
import numpy as np
import time

from rclpy.node import Node
import rclpy.action
import rclpy.task
import rclpy.executors
import rclpy.callback_groups


from rclpy.action import ActionClient, ActionServer
from sentinel_msgs.action import FollowPath
from robomaster_msgs.action import Move
from geometry_msgs.msg import PoseStamped, Pose

from math import pi
from sentinel.utils import frame_from_pose, pose_from_frame, transform_from_state


def wait_for_completion(future):
    state = {'done': False,'rejected':False}

    def result_cb(future):
        state['result'] = future.result().result
        state['done'] = True

    def accepted_cb(future):
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_cb)
        else:
            state['rejected'] = True

    future.add_done_callback(accepted_cb)

    while not state['done']:
        time.sleep(0.1)

    return state.get('result')


class Follow_Path(Node):

    def __init__(self):
        super().__init__('followPath_action')

        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.action_server = ActionServer(self, FollowPath, 'followPath', self.server_callback, cancel_callback=self.cancel_callback, callback_group=cbg)

        self.action_client = ActionClient(self, Move, 'move')

        self.return_orientation = self.declare_parameter('return_orientation', 0).value

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 1)
        self.pose = Pose()

        self.cancel_flag = False

    def cancel_callback(self, request):
        self.cancel_flag = True

        return rclpy.action.server.CancelResponse.ACCEPT

    def pose_callback(self, msg):
        self.noisy_pose = msg

    def server_callback(self, goal_handle):
        path = goal_handle.request
        done = False

        feedback_msg = FollowPath.Feedback()
        
        path_size = len(path.path.poses)
        trajectory = path.path.poses
        linear_speed = path.linear_speed
        angular_speed = path.angular_speed
        following = path.following
        
        while not done:

            path_target = trajectory.pop(0)

            path_theta = 2 * np.arctan2(path_target.pose.orientation.z, path_target.pose.orientation.w)

            # turn 180 to go back to base
            if len(path.path.poses) != 0 and self.return_orientation == 0 and not following:
                path_theta += pi

            # obtain the target pose with respect to the current robot frame
            current_pose = frame_from_pose(self.noisy_pose.pose)
            target_pose = transform_from_state(path_target.pose.position.x, path_target.pose.position.y, path_theta)
            next_pose = pose_from_frame(current_pose.Inverse() * target_pose)

            n_theta = 2 * np.arctan2(next_pose.orientation.z, next_pose.orientation.w)

            # send the target pose the action client server and wait for the goal to be reached if the goal is not cancelled
            if self.cancel_flag:
                done = True
                self.cancel_flag = False
            else:
                self.move(next_pose, n_theta, linear_speed, angular_speed)

            feedback_msg.progress = 1 - (len(trajectory)/path_size)

            # publish feedback
            goal_handle.publish_feedback(feedback_msg)

            if len(path.path.poses) == 0:
                done = True

        result = FollowPath.Result()

        goal_handle.succeed()
        return result
    
    def move(self, next_pose, theta, linear_speed, angular_speed):
        goal_msg = Move.Goal()
        goal_msg.x = next_pose.position.x
        goal_msg.y = next_pose.position.y
        goal_msg.theta = theta
        goal_msg.linear_speed = linear_speed
        goal_msg.angular_speed = angular_speed
        
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        wait_for_completion(future)

        return True
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback}')
    
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = Follow_Path()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
