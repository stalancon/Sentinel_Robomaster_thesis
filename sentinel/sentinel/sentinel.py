import rclpy
import std_msgs.msg
import numpy as np

from math import dist
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

class Sentinel(Node):

    def __init__(self):
        super().__init__('sentinel')

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 20)
        self.pose = Pose()

        # Anomaly subscriber
        self.create_subscription(Bool,'anomaly', self.listener_callback, 10)

        # led publisher        
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 10)
        
        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        # Action client
        self.action_client = ActionClient(self, FollowPath, 'followPath')

        self.poses_path = Path()
        self.flag = False
        self.moving_flag = False

        self.counter = 0

        self.path_to_follow = Path()
        next_pose = PoseStamped()
        next_pose.header.frame_id = 'world'

        poses_path = [[0.5, -1.5, 0.0], [1.6, -1.35, 0.34], [1.9, -0.7, 1.0] ,[1.9, 0.0, 1.56], [1.9, 1.5, 1.9], [1.5, 1.9, 2.7], [0.5, 1.9, 3.14], [-0.9, 1.9, 3.14]]

        pose_array = []

        for i in poses_path:
            next_pose = PoseStamped()
            next_pose.pose.position.x = i[0]
            next_pose.pose.position.y = i[1]
            next_pose.pose.orientation.w = np.cos(i[2]/2)
            next_pose.pose.orientation.z = np.sin(i[2]/2)
            pose_array.append(next_pose)

        self.path_to_follow.poses = pose_array

    def pose_callback(self, noisy_pose):
        
        if self.flag:
            led_effect = LEDEffect.ON
            led_color = red
        else:
            led_effect = LEDEffect.ON
            led_color = green

        self.led_pub.publish(LEDEffect(effect=led_effect, color=led_color))

        if not self.moving_flag and self.counter == 100: 
            self.path_to_follow.header.frame_id = noisy_pose.header.frame_id
            self.send_goal(self.path_to_follow)
            self.moving_flag = True

        self.counter += 1

    def send_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.linear_speed = 0.1
        goal_msg.following = True

        self.action_client.wait_for_server()

        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        num = feedback.progress

        if num == 1:
            # self.moving_flag = False
            self.get_logger().info(f'End of the path')

    def listener_callback(self, msg):
        if msg.data:
            self.flag = True
            self.stop()

    def stop(self):
        self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=black))
        self.velocity = Twist()
        self.velocity_pub.publish(self.velocity)

def main(args=None):

    rclpy.init(args=args)
    node = Sentinel()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # node.destroy_node()
    rclpy.shutdown()

