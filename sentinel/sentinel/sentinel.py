import rclpy
import numpy as np
import PyKDL
import math

from math import dist, pi
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, ColorRGBA
from robomaster_msgs.msg import LEDEffect
from sentinel_msgs.msg import Radio, PathMsg
from shapely.geometry import Point, LineString
from shapely.ops import split
from rcl_interfaces.msg import SetParametersResult

from rclpy.action import ActionClient
from sentinel_msgs.action import FollowPath
from sentinel.utils import distance_delta


green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.0)
black = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)
yellow = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.0)

class Sentinel(Node):

    def __init__(self):
        super().__init__('sentinel')

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 10)
        self.pose = Pose()

        # Anomaly subscriber
        self.create_subscription(Bool,'/anomaly', self.listener_callback, 10)

        # Malfunction subscriber
        self.create_subscription(Bool,'/trigger', self.malfunction_callback, 10)

        # Path subscriber
        self.create_subscription(Path,'/path', self.path_callback, 10)

        # led publisher        
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 10)
        
        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velocity = Twist()

        # radio subscriber & publisher
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

        self.path_to_follow = Path()
        self.poses_path = Path()
        self.start_flag = True
        self.path_updated = False
        self.follower_flag = False
        self.path_received = False
        self.double_path = False
        self.autonomy_flag = False
        self.once = True


        self.counter = 0
        self.state = 7
        self.status = 'Starting'


        if self.mov_node == 'action_client':
            # action client
            self.action_client = ActionClient(self, FollowPath, 'followPath')

            self.followPath_goal_handle = None

        elif self.mov_node == 'speed_controller':
            # SPEED CONTROLLER
            
            # path message for speed_controller publisher
            self.path_pub = self.create_publisher(PathMsg,'path_trajectory', 10)

            self.anomaly_check = True


        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def path_callback(self, path):

        if not self.path_received:
            self.path_to_follow = path
            self.path_received = True
        else:
            if self.path_to_follow.poses != path.poses:
                self.chosen_path = self.create_line_object(self.path_to_follow)
                self.backup_path = self.create_line_object(path)
                self.double_path = True


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


    def construct_path(self, path):
        pose_array = []
        next_pose_check = False

        for i, pose in enumerate(path):
            path_check = False
            next_pose = PoseStamped()
            next_pose.pose.position.x = pose[0]
            next_pose.pose.position.y = pose[1]
            n_point = [pose[0], pose[1]]

            for j in self.path1_poses:
                if n_point == j[:-1]:

                    if j[-1] == 0.0:
                        theta = j[-1] + pi
                    else:
                        theta = -j[-1]
                    next_pose.pose.orientation.w = np.cos(theta/2)
                    next_pose.pose.orientation.z = np.sin(theta/2)
                    path_check = True
            for j in self.path2_poses:
                if n_point == j[:-1]:
                    next_pose.pose.orientation.w = np.cos(j[2]/2)
                    next_pose.pose.orientation.z = np.sin(j[2]/2)
                    path_check = True
            if not path_check:
                try:
                    theta = math.atan((pose[1] - path[i-1][1]) / (pose[0] - path[i-1][0]))
                    if theta < 0.5 and theta > -0.5:
                        theta = pi
                except:

                    if path[i-1][1] < pose[1]:
                        theta = pi/2
                    else:
                        theta = -   pi/2

                # self.get_logger().info(f'p: {n_point}')
                # self.get_logger().info(f't: {theta}')

                next_pose.pose.orientation.w = np.cos(theta/2)
                next_pose.pose.orientation.z = np.sin(theta/2)

            pose_array.append(next_pose)

        return pose_array
            

    def timer_callback(self):
        try:
            self.radio_pub.publish(Radio(source=1, state=self.state, pose=self.current_pose))
        except:
            pass


    def create_line_object(self, path):
        poses = path.poses
        
        line_array = []

        for p in poses:
            point = (p.pose.position.x, p.pose.position.y)
            line_array.append(point)

        path_line = LineString(line_array)

        return path_line
        

    def readjust_path(self, follower_pose):
        self.stop()

        current_point = Point(self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        # self.get_logger().info(f'current pose: {current_point}')

        # Add buffer zone to the location of the follower
        follower_loc = Point(follower_pose.pose.position.x, follower_pose.pose.position.y).buffer(1)

        # Obtain the path done by the sentinel so far
        end_point = self.chosen_path.interpolate(self.chosen_path.project(current_point))

        x = split(self.chosen_path, end_point)
        self.chosen_path = x.geoms[0]

        # Obtain the points of the intersection between the path done and the buffer zone for the follower
        inter_path = self.chosen_path.intersection(follower_loc)

        inter_start = Point(inter_path.coords[0][0], inter_path.coords[0][1])
        inter_end = Point(inter_path.coords[-1][0], inter_path.coords[-1][1])


        # Move the intersecting part of the path 0.4 distance to the left
        new_section = inter_path.parallel_offset(0.3, 'left', join_style=1)

        # Split the line according to the intersecting points
        first_part = split(self.chosen_path, inter_start)
        second_part = split(self.chosen_path, inter_end)
        
        new_end = first_part.geoms[0]

        try:
            new_start = second_part.geoms[1]
        except:
            new_start = new_end

                                
        intersection_line = LineString(inter_end.coords[:] + new_section.coords[::-1] + inter_start.coords[:])

        # create more points on the intersection line
        num_points = 7
        distances = np.linspace(0, intersection_line.length, num_points)
        points = [intersection_line.interpolate(distance) for distance in distances]
        new_intersection = LineString(points)

        new_path = LineString(new_start.coords[::-1] + new_intersection.coords[:] + new_end.coords[::-1])

        # obtain the unique parts of both paths to create new path
        paths = new_path.symmetric_difference(self.backup_path)

        start = paths.geoms[0]
        end = paths.geoms[-1]

        new_path_line = LineString(start.coords[:] + end.coords[:])

        self.new_path.poses = self.construct_path(list(new_path_line.coords))

        self.turn_check = False
        self.path_updated = True

    def radio_callback(self, radio_msg):
        # Only take into account the messages from the follower robot
        if radio_msg.source == 2:
            follower_pose = radio_msg.pose
            follower_state = radio_msg.state

            if follower_state == 0:
                self.follower_flag = True

            if self.mov_node == 'speed_controller':

                # Keep track of safety gap
                if self.state != 1 and not self.start_flag and self.state != 2 and self.path_received:

                    delta_dist, delta_theta = distance_delta(follower_pose, self.current_pose)

                    if delta_dist > (self.gap_dist * 2):
                        self.state = 5
                    elif delta_dist > self.gap_dist or delta_theta > self.gap_theta or follower_state == 3:
                        self.state = 4  # slow down
                    else:
                        self.state = 0  # normal speed 
                    
                    self.path_pub.publish(PathMsg(path=self.path_to_follow, state=self.state, linear_speed=self.linear_speed, angular_speed=self.angular_speed))
                


                # Readjust path to avoid hazard
                elif self.state == 1 and self.double_path:
                    if not self.path_updated:
                        self.readjust_path(follower_pose)

    def pose_callback(self, noisy_pose):
        
        self.current_pose = noisy_pose

        if self.state == 1: # Check if there's been an anomaly
            led_effect = LEDEffect.ON
            led_color = red

            if self.mov_node == 'speed_controller':
                # SPEED CONTROLLER
                if self.anomaly_check and self.path_received:
                    self.path_pub.publish(PathMsg(path=self.path_to_follow, state=self.state, linear_speed=self.linear_speed, angular_speed=self.angular_speed))
                    self.anomaly_check = False

        elif self.state == 5 or self.state == 7:
            led_effect = LEDEffect.ON
            led_color = yellow

        else:
            led_effect = LEDEffect.ON
            led_color = green

        self.led_pub.publish(LEDEffect(effect=led_effect, color=led_color))

        if self.follower_flag and not self.path_received and self.state == 7:
            self.state = 0

        # Send the path to the movement node chosen to start the path
        if self.follower_flag and self.once and self.path_received: 
            self.start_flag = False
            self.state = 0
            self.get_logger().info('Lets go!!')
            self.path_to_follow.header.frame_id = noisy_pose.header.frame_id

            if self.mov_node == 'action_client':
                self.send_goal(self.path_to_follow)

            elif self.mov_node == 'speed_controller':
                self.path_pub.publish(PathMsg(path=self.path_to_follow, state=self.state, linear_speed=self.linear_speed, angular_speed=self.angular_speed))

            self.once = False


        if self.double_path and self.state == 1:       
            if not self.turn_check:

                if self.turn_counter < 125:
                    self.turn()
                    self.counter += 1
                else:
                    self.stop()
                    self.turn_check = True
                    self.state = 2

                    self.path_pub.publish(PathMsg(path=self.new_path, state=self.state, linear_speed=self.linear_speed, angular_speed=self.angular_speed))
                    self.get_logger().info('path was sent')

                self.turn_counter += 1 

        self.counter += 1

    def send_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.linear_speed = self.linear_speed
        goal_msg.angular_speed = self.angular_speed
        goal_msg.following = True

        self.action_client.wait_for_server()

        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        def result_cb(future):
            self.followPath_goal_handle = None

        def accepted_cb(future):
            goal_handle = future.result()
            if goal_handle.accepted:
                self.followPath_goal_handle = goal_handle
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_cb)

        future.add_done_callback(accepted_cb)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        num = feedback.progress

        if num == 1:
            self.get_logger().info(f'End of the path')

    def listener_callback(self, msg):
        if msg.data:
            self.state = 1
            self.stop()

            if self.mov_node == 'action_client':
                # ACTION CLIENT
                if self.followPath_goal_handle is not None:
                    self.get_logger().info('An anomaly was heard! The goal has been cancelled')
                    self.followPath_goal_handle.cancel_goal_async()
                    self.followPath_goal_handle = None


    def malfunction_callback(self, msg):
        self.state = 6
        self.stop()
        self.path_pub.publish(PathMsg(path=self.path_2, state=self.state))


    def turn(self):
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.z = 0.55
        self.velocity_pub.publish(self.velocity)


    def stop(self):
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

