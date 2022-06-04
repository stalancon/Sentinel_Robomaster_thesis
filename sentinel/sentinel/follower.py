

import rclpy
import numpy as np

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from robomaster_msgs.msg import LEDEffect
from sentinel_msgs.msg import Radio, PathMsg
from rclpy.action import ActionClient
from sentinel_msgs.action import FollowPath
from rcl_interfaces.msg import SetParametersResult
from sentinel.utils import distance_delta

blue = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.0)
black = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)
yellow = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.0)

class Follower(Node):

    def __init__(self):
        super().__init__('follower')

        # LED publisher        
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 10)
        
        # Noisy pose subscriber
        self.create_subscription(PoseStamped, 'noisy_pose', self.pose_callback, 10)
        self.pose = Pose()

        # Velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity = Twist()

        # Radio subscriber & publisher
        self.create_subscription(Radio,'/radio', self.radio_callback, 4)
        self.radio_pub = self.create_publisher(Radio,'/radio', 4)

        # Parameter to chose the node for moving the robot
        self.mov_node = self.declare_parameter('movement_node', 'speed_controller').value

        # Parameter to chose the gap to keep between robots
        self.gap_dist = self.declare_parameter('gap_dist', 0.5).value

        # Parameter to chose the delta gap to keep between robots
        self.gap_theta = self.declare_parameter('gap_theta', 0.52).value

        # Speed 
        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value

        self.add_on_set_parameters_callback(self.cb_params)

        self.follower_path = Path()
        self.sentinel_path = Path()

        self.sentinel_pose = PoseStamped()

        self.status = 'Starting'

        # flags
        self.flag = False
        self.anomaly_check = True
        self.onetime_check = True
        self.start_check = True

        self.state = 7

        if self.mov_node == 'action_client':
            # ACTION CLIENT

            # Action client
            self.action_client = ActionClient(self, FollowPath, 'followPath')

            # flag to indicate the robot sent a follow path action
            self.moving_flag = False

            self.followPath_goal_handle = None

        elif self.mov_node == 'speed_controller':
            # SPEED CONTROLLER

            # path message for followspeed_controller publisher
            self.follow_pub = self.create_publisher(PathMsg,'follow_trajectory', 10)

            # path message for speed_controller publisher
            self.path_pub = self.create_publisher(PathMsg,'path_trajectory', 10)

            self.sentinel_goals = []

            self.counter = 0
            self.once = True

        self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=yellow))

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def cb_params(self, data):

        for param in data:
            if param.name == 'linear_speed':
                self.linear_speed = param.value
            elif param.name == 'angular_speed':
                self.angular_speed = param.value
            elif param.name == 'gap_dist':
                self.gap_dist = param.value
            elif param.name == 'gap_theta':
                self.gap_theta = param.value
        return SetParametersResult(successful=True)

    def timer_callback(self):
        try:
            self.radio_pub.publish(Radio(source=2, state=self.state, pose=self.current_pose))
        except:
            pass

    def radio_callback(self, radio_msg):
        # Only take into account the messages from the sentinel robot
        if radio_msg.source == 1:
            sentinel_pose = radio_msg.pose
            sentinel_state = radio_msg.state
            
            if sentinel_state == 1:  # check if there's an anomaly
                if self.anomaly_check:
                    self.flag = True

                    if self.mov_node == 'action_client':
                        # ACTION CLIENT
                        self.last_pose_sentinel = sentinel_pose
                        self.stop()
                        self.status = 'Cancelled'
                        if self.followPath_goal_handle is not None and self.status != 'Base':
                            self.get_logger().info('An anomaly was heard! The goal has been cancelled')
                            self.followPath_goal_handle.cancel_goal_async()
                            self.followPath_goal_handle = None
                    elif self.mov_node == 'speed_controller':
                        # SPEED CONTROLLER
                        self.follow_pub.publish(PathMsg(path=self.sentinel_path, state=1, linear_speed=self.linear_speed, angular_speed=self.angular_speed))
                        self.status = 'Turning'

                    self.anomaly_check = False
            
            elif sentinel_state != 7:

                if self.mov_node == 'speed_controller':
                    # SPEED CONTROLLER
                    
                    # speed adjustment depending on the position of the sentinel
                    if self.status == 'Following':
                        delta_dist, delta_theta = distance_delta(self.current_pose, sentinel_pose)

                        if delta_dist > (self.gap_dist * 2):
                            self.state = 3  # speed up
                        elif delta_dist < 0.5:
                            self.state = 5
                            self.get_logger().info('too close!')
                        elif delta_dist < self.gap_dist:
                            self.state = 4  # slow down
                        else:
                            self.state = 0  # normal speed 
                        
                        self.follow_pub.publish(PathMsg(path=self.sentinel_path, state=self.state, linear_speed=self.linear_speed, angular_speed=self.angular_speed))


                if self.start_check:
                    self.status = 'Following'
                    self.get_logger().info('Lets go!')
                    self.start_check = False

                # Create/Update sentinel path
                if len(self.sentinel_path.poses) == 0:
                    self.sentinel_path.header.frame_id = sentinel_pose.header.frame_id
                    self.sentinel_path.poses.append(sentinel_pose)
                else: 
                    past_pose = self.sentinel_path.poses[-1]
                    delta_dist, delta_theta = distance_delta(past_pose, sentinel_pose)

                    if delta_dist > self.gap_dist or abs(delta_theta) > self.gap_theta:
                        self.sentinel_path.poses.append(sentinel_pose)
                        self.last_pose_sentinel = self.sentinel_path.poses[-1]



    def pose_callback(self, noisy_pose):

        self.current_pose = noisy_pose

        if self.status == 'Starting':
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=yellow))
            self.state = 0


        if self.status == 'Following':
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=blue))

            if len(self.follower_path.poses) == 0:
                self.follower_path.header.frame_id = noisy_pose.header.frame_id
                self.follower_path.poses.append(noisy_pose)
            
            try:
                sentinel_pose = self.sentinel_path.poses[-1]
            except:
                sentinel_pose = self.last_pose

            delta_dist, delta_theta = distance_delta(sentinel_pose, noisy_pose)

            if delta_dist > self.gap_dist or abs(delta_theta) > self.gap_theta:
                
                if self.mov_node == 'action_client':
                    # ACTION CLIENT
                    if not self.moving_flag:
                        self.follower_path.header.frame_id = noisy_pose.header.frame_id
                        self.follower_path.poses.insert(0, noisy_pose)

                        self.send_goal(self.sentinel_path)

                        self.moving_flag = True
                        self.last_pose = self.sentinel_path.poses[-1]
                        self.sentinel_path.poses = []

                    else:
                        past_pose = self.follower_path.poses[0]
                        
                        delta_dist, delta_theta = distance_delta(past_pose, noisy_pose)

                        if delta_dist > self.gap_dist or abs(delta_theta) > self.gap_theta:
                            self.follower_path.poses.insert(0, noisy_pose)

                elif self.mov_node == 'speed_controller':
                    # SPEED CONTROLLER
                    if sentinel_pose not in self.sentinel_goals:
                        self.sentinel_goals.append(sentinel_pose)
                        self.follower_path.poses.insert(0, noisy_pose)
                    self.follow_pub.publish(PathMsg(path=self.sentinel_path, state=self.state, linear_speed=self.linear_speed, angular_speed=self.angular_speed))


        if self.flag:
            self.led_pub.publish(LEDEffect(effect=LEDEffect.ON, color=red))

            if self.mov_node == 'action_client':
                # ACTION CLIENT
                if not self.moving_flag:
                    self.stop()
                    self.follower_path.poses.insert(0, noisy_pose)
                
                    self.status = 'Base'

                    self.send_goal(self.follower_path)
                    self.get_logger().info('Going back to base')

                    self.moving_flag = True

            elif self.mov_node == 'speed_controller':
                # SPEED CONTROLLER
                if self.status == 'Turning':

                    # if self.counter < 420:
                    if self.counter < 110:
                        self.turn()
                        self.counter += 1
                    else:
                        self.stop()
                        self.status = 'Base'

                elif self.onetime_check and self.status == 'Base':

                    self.follower_path.poses = self.follower_path.poses[1:]
                    self.path_pub.publish(PathMsg(path=self.follower_path, state = 2, linear_speed=self.linear_speed, angular_speed=self.angular_speed))

                    self.onetime_check = False

    def send_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.linear_speed = self.linear_speed
        goal_msg.angular_speed = self.angular_speed

        if self.status == 'Base':
            goal_msg.following = False
        else:
            goal_msg.following = True

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

        if num == 1 or self.status == 'Cancelled':
            self.moving_flag = False

            if self.status == 'Base':
                
                self.flag = False
                self.get_logger().info(f'Back at base')
                self.follower_path = Path()
        

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
    node = Follower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # node.destroy_node()
    rclpy.shutdown()

