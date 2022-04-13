from selectors import EpollSelector
from unittest.mock import sentinel
import rclpy
import std_msgs.msg
import numpy as np

from rclpy.node import Node
from math import dist, floor
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from robomaster_msgs.msg import LEDEffect
from rclpy.action import ActionClient
from sentinel_msgs.action import FollowPath

blue = std_msgs.msg.ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.0)
black = std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
red = std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        
        # noisy pose subscriber
        self.create_subscription(PoseStamped, 'noisy_pose', self.pose_callback, 10)
        self.pose = Pose()

        # Sentinel noisy pose subscriber
        self.create_subscription(PoseStamped, '/rm_1/noisy_pose', self.sentinel_pose_callback, 10)

        # Anomaly subscriber
        self.create_subscription(Bool,'/rm_1/anomaly', self.listener_callback, 10)

        # led publisher        
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 10)
        
        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        # Action client
        self.action_client = ActionClient(self, FollowPath, 'followPath')

        self.follower_path = Path()
        self.sentinel_path = Path()

        self.flip_path = Path()

        self.sentinel_pose = PoseStamped()

        self.status = 'Following'

        # flag to indicate an anomaly
        self.flag = False

        # flag to indicate the robot sent a follow path action
        self.moving_flag = False

    def sentinel_pose_callback(self, noisy_pose):

        if not self.flag:

            if len(self.sentinel_path.poses) == 0:
                self.sentinel_path.header.frame_id = noisy_pose.header.frame_id
                self.sentinel_path.poses.append(noisy_pose)

            else: 
                past_pose = self.sentinel_path.poses[-1]

                delta_dist, delta_theta = self.distance_delta(past_pose, noisy_pose)

                if delta_dist > 1 or abs(delta_theta) > 0.78:
                    self.sentinel_path.header.frame_id = noisy_pose.header.frame_id
                    self.sentinel_path.poses.append(noisy_pose)

                    self.last_pose_sentinel = self.sentinel_path.poses[-1]

        if self.flag:
            if self.last_pose_sentinel == noisy_pose:
                self.status = 'Stopped'

    def pose_callback(self, noisy_pose):

        if not self.flag:
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=blue))

        if self.status == 'Following':

            if len(self.follower_path.poses) == 0:
                self.follower_path.header.frame_id = noisy_pose.header.frame_id
                self.follower_path.poses.append(noisy_pose)
            else:
                
                try:
                    sentinel_pose = self.sentinel_path.poses[-1]
                except:
                    sentinel_pose = self.last_pose

                delta_dist, delta_theta = self.distance_delta(sentinel_pose, noisy_pose)

                if delta_dist > 1 or abs(delta_theta) > 0.52:

                    if not self.moving_flag:
                        self.follower_path.header.frame_id = noisy_pose.header.frame_id
                        self.follower_path.poses.insert(0,noisy_pose)

                        # self.flip_path.header.frame_id = noisy_pose.header.frame_id
                        # self.flip_path.poses = self.sentinel_path.poses[::-1]

                        # self.send_goal(self.flip_path)
                        self.send_goal(self.sentinel_path)

                        self.moving_flag = True

                        self.last_pose = self.sentinel_path.poses[-1]

                        self.sentinel_path.poses = []

                    else:
                        past_pose = self.follower_path.poses[0]
                        
                        delta_dist, delta_theta = self.distance_delta(past_pose, noisy_pose)

                        if delta_dist > 1 or abs(delta_theta) > 0.52:
                            self.follower_path.poses.insert(0, noisy_pose)

        if self.flag:
            if not self.moving_flag:
                self.stop()
                self.follower_path.poses.insert(0,noisy_pose)

                self.get_logger().info('Anomaly detected.. going back to base')
            
                self.status = 'Base'

                self.send_goal(self.follower_path)

                self.moving_flag = True
            

    def distance_delta(self, past_pose, current_pose):
        current = [current_pose.pose.position.x, current_pose.pose.position.y]
        past = [past_pose.pose.position.x, past_pose.pose.position.y]

        delta_dist = dist(past, current)

        past_theta = 2 * np.arctan2(past_pose.pose.orientation.z, past_pose.pose.orientation.w)
        theta = 2 * np.arctan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w)

        delta_theta = theta - past_theta

        return delta_dist, delta_theta

    def send_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.linear_speed = 0.8

        if self.status == 'Base':
            goal_msg.following = False
        else:
            goal_msg.following = True

        self.action_client.wait_for_server()

        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        num = feedback.progress

        if num == 1:
            self.moving_flag = False

            if self.status == 'Base':

                self.flag = False
                self.get_logger().info(f'Back at base')
                self.follower_path = Path()

        if self.status == 'Base':
            i = min(7, floor(num * 8))
            g_mask = 2 ** (i + 1) - 1
    
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, submask=g_mask, color=red))


    def listener_callback(self, msg):
        if msg.data:
            self.flag = True
            self.stop()
    
    def stop(self):
        self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, submask=255, color=black))
        self.velocity = Twist()
        self.velocity_pub.publish(self.velocity)
    

def main(args=None):

    rclpy.init(args=args)
    node = Follower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # node.destroy_node()
    rclpy.shutdown()

