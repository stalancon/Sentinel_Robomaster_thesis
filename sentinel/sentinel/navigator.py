import rclpy
import std_msgs.msg

from math import floor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from robomaster_msgs.msg import LEDEffect
from rclpy.action import ActionClient
from sentinel_msgs.action import FollowPath
from sentinel_msgs.msg import PathMsg
from sentinel.utils import distance_delta

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
        self.create_subscription(Bool,'/anomaly', self.listener_callback, 10)

        # led publisher        
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 1)

        # Path subscriber
        self.create_subscription(Path,'/path', self.path_callback, 10)
        
        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        # Parameter to chose the node for moving the robot
        self.mov_node = self.declare_parameter('movement_node', 'speed_controller').value

        # Speed 
        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value

        if self.mov_node == 'action_client':

            # Action client
            self.action_client = ActionClient(self, FollowPath, 'followPath')

        elif self.mov_node == 'speed_controller':

            # path message for speed_controller publisher
            self.path_pub = self.create_publisher(PathMsg,'path_trajectory', 10)

            self.status = 'Driving'
            self.counter = 0

        self.path_received = False

        # flag to indicate an anomaly has been heard
        self.flag = False
        self.check = False
        self.once = True
        self.autonomy_flag = False

        self.poses_path = Path()

    def path_callback(self, path):

        if not self.path_received:
            self.path_to_follow = path
            self.path_received = True
            self.autonomy_flag = True

    def pose_callback(self, noisy_pose):

        # Send the path to the movement node chosen to start the path
        if self.path_received and self.once: 

            self.get_logger().info('Lets go!!')
            self.path_to_follow.header.frame_id = noisy_pose.header.frame_id

            if self.mov_node == 'action_client':
                self.send_goal(self.path_to_follow)

            elif self.mov_node == 'speed_controller':
                self.path_pub.publish(PathMsg(path=self.path_to_follow, state=0, linear_speed=self.linear_speed, angular_speed=self.angular_speed))

            self.once = False

        
        # Record the path 
        if not self.flag:
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, mask=63, color=green))

            if len(self.poses_path.poses) == 0:
                self.poses_path.header.frame_id = noisy_pose.header.frame_id
                self.poses_path.poses.append(noisy_pose)

            else:
                past_pose = self.poses_path.poses[0]

                delta_dist, delta_theta = distance_delta(past_pose, noisy_pose)

                if delta_dist > 1.0 or abs(delta_theta) > 0.52:
                    self.poses_path.header.frame_id = noisy_pose.header.frame_id
                    self.poses_path.poses.insert(0, noisy_pose)


        # Anomaly heard
        if self.flag: 
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=red))

            if self.mov_node == 'action_client':
                # ACTION CLIENT
                if not self.check:
                    self.stop()
                    self.poses_path.poses.insert(0, noisy_pose)

                    self.get_logger().info('Anomaly detected.. going back to base')

                    self.send_goal(self.poses_path)

                    self.check = True

            elif self.mov_node == 'speed_controller':
                # SPEED CONTROLLER
                if self.status != 'Base':

                    if self.counter < 115:
                        self.turn()
                        self.counter += 1
                    else:
                        self.stop()
                        self.status = 'Base'

                elif not self.check and self.status == 'Base':
                    self.get_logger().info('Anomaly detected.. going back to base')

                    self.path_pub.publish(PathMsg(path=self.poses_path, state=2, linear_speed=self.linear_speed, angular_speed=self.angular_speed))

                    self.check = True

    def send_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.linear_speed = self.linear_speed
        goal_msg.angular_speed = self.angular_speed

        goal_msg.following = False

        self.action_client.wait_for_server()

        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        def result_cb(future):
            self.followPath_goal_handle = None

        def accepted_cb(future):
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Goal was accepted!!')
                self.followPath_goal_handle = goal_handle
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_cb)

        future.add_done_callback(accepted_cb)
    
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

    def listener_callback(self, msg):

        # if we hear 'True' we have an anomaly
        if msg.data:
            self.flag = True

            if self.autonomy_flag and self.mov_node == 'speed_controller':
                self.path_pub.publish(PathMsg(path=self.path_to_follow, state=1, linear_speed=self.linear_speed, angular_speed=self.angular_speed))


    def turn(self):
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.z = 0.6
        self.velocity_pub.publish(self.velocity)

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

