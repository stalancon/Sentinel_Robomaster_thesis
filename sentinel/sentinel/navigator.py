import rclpy
import std_msgs.msg
import numpy as np

from math import floor, dist
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from robomaster_msgs.msg import LEDEffect
from rclpy.action import ActionClient
from sentinel_msgs.action import FollowPath

green = std_msgs.msg.ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.0)
black = std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
red = std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 20)
        self.pose = Pose()

        # Anomaly subscriber
        self.create_subscription(Bool,'anomaly', self.listener_callback, 10)

        # led publisher        
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 1)
        
        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        # Path publisher
        # self.path_publisher = self.create_publisher(Path,'path', 10)

        # Action client
        self.action_client = ActionClient(self, FollowPath, 'followPath')

        # flag to indicate an anomaly has been heard
        self.flag = False
        self.check = False

        self.poses_path = Path()

    def send_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.linear_speed = 0.55

        self.action_client.wait_for_server()

        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        num = feedback.progress

        i = min(7, floor(num * 8))
        g_mask = 2 ** (i + 1) - 1

        if num == 1:
            self.flag = False
            self.get_logger().info(f'Back at base')
            self.check = False
            self.poses_path = Path()
    
        self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, submask=g_mask, color=red))

    def pose_callback(self, noisy_pose):

        if not self.flag:
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, mask=63, color=green))

            if len(self.poses_path.poses) == 0:
                self.poses_path.header.frame_id = noisy_pose.header.frame_id
                self.poses_path.poses.append(noisy_pose)

            else:
                past_pose = self.poses_path.poses[-1]

                p = [past_pose.pose.position.x, past_pose.pose.position.y]
                current_pose = [noisy_pose.pose.position.x, noisy_pose.pose.position.y]

                delta_dist = dist(p, current_pose)

                past_theta = 2 * np.arctan2(past_pose.pose.orientation.z, past_pose.pose.orientation.w)
                theta = 2 * np.arctan2(noisy_pose.pose.orientation.z, noisy_pose.pose.orientation.w)

                delta_theta = theta - past_theta

                if delta_dist > 1 or abs(delta_theta) > 0.26:
                    self.poses_path.header.frame_id = noisy_pose.header.frame_id
                    self.poses_path.poses.append(noisy_pose)

        if self.flag: 
            if not self.check:
                self.stop()

                self.get_logger().info('Anomaly detected.. going back to base')
                
                self.send_goal(self.poses_path)

                self.check = True


    def listener_callback(self, msg):

        # if we hear 'True' we have an anomaly
        if msg.data:
            self.flag = True

    def stop(self):
        self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, submask=255, color=black))
        self.velocity = Twist()
        self.velocity_pub.publish(self.velocity)

def main(args=None):

    rclpy.init(args=args)
    node = Navigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # node.destroy_node()
    rclpy.shutdown()

