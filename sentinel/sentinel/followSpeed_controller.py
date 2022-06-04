

import rclpy
import numpy as np
import math 

from math import pi
from rclpy.node import Node
from shapely.geometry import LineString, Point
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sentinel_msgs.msg import PathMsg
from sentinel.utils import compute_theta


class FollowSpeed_controller(Node):

    def __init__(self):
        super().__init__('followspeed_controller')
        
        # path subscriber
        self.create_subscription(PathMsg, 'follow_trajectory', self.path_callback, 10)

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 10)
        self.pose = Pose()

        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        self.path_created = False
        self.onetime_check = True

        self.status = 'start'
        self.counter = 0

        self.path_done = []

    def pose_callback(self, noisy_pose):

        if self.path_created:

            if not self.status == 'done':

                pose_point = Point(noisy_pose.pose.position.x, noisy_pose.pose.position.y)

                # Get the point in the line that's closest to the current pose
                goal_pose = self.path_line.interpolate(self.path_line.project(pose_point))

                # self.get_logger().info(f'goal p: {goal_pose}')

                # Get distance between the current position and the goal pose
                dist_goal = goal_pose.distance(pose_point)

                if dist_goal < 0.1 and len(self.path_line.coords) > 2:
                    # If we are close enough to the goal pose, eliminate pose from path 
                    for i, p in enumerate(list(self.path_line.coords)):
                        if i == 0 and Point(p) == goal_pose:
                            self.path_line = LineString(self.path_line.coords[i+1:]) 
                            self.past_theta = self.theta_array[0]
                            self.theta_array = self.theta_array[i+1:] 
                            self.path_done.append(p) 
                else:

                    # If we passed the goal pose, eliminate pose from path and change goal pose to next pose in path
                    if len(self.path_line.coords) > 2:
                        for i, p in enumerate(list(self.path_line.coords)):
                            if i == 0 and Point(p) != goal_pose:
                                goal_pose = Point(self.path_line.coords[i+1])
                                self.path_line = LineString(self.path_line.coords[i+1:]) 
                                self.past_theta = self.theta_array[0]
                                self.theta_array = self.theta_array[i+1:]
                                self.path_done.append(p)

                    # Unit vector
                    vector = [goal_pose.x - pose_point.x, goal_pose.y - pose_point.y]
                    unit_vector = vector / np.linalg.norm(vector)

                    target_theta = self.theta_array[0]
                    current_theta = compute_theta(noisy_pose)

                    side = self.past_theta - target_theta

                    current_theta = math.fmod(current_theta, 2*pi)
                    target_theta = math.fmod(target_theta, 2*pi)

                    # if current_theta > pi:
                    #     current_theta = pi - current_theta
                    # elif current_theta < -pi:
                    #     current_theta = current_theta + pi
                    
                    # if target_theta > pi:
                    #     target_theta = pi - target_theta
                    # elif target_theta < -pi:
                    #     target_theta = target_theta + pi

                    theta = target_theta - current_theta

                    if side > -0.1 and side < 0.1: # check if i need to turn

                        if abs(theta) > 0.2:
                            self.velocity.angular.z = 0.35 * -theta
                        else:
                            self.velocity.angular.z = 0.0
                    else:
                        if side < 0.0:
                            # turn left
                            theta = -theta
                        else:
                            # turn right
                            theta = abs(theta)

                        self.velocity.angular.z = self.angular_speed * theta
                    
                    if abs(vector[1]) > abs(vector[0]):
                        self.velocity.linear.x = self.linear_speed * abs(unit_vector[1])
                    else:
                        self.velocity.linear.x = self.linear_speed * abs(unit_vector[0])

                    self.velocity_pub.publish(self.velocity)


    def create_pathLine(self, path_data):
        
        line_array = []
        self.theta_array = []

        for pose in path_data.poses:
            point_x = pose.pose.position.x
            point_y = pose.pose.position.y
            point = (point_x, point_y)
            line_array.append(point)

            theta = compute_theta(pose)

            self.theta_array.append(theta)

        self.path_array = line_array

        self.path_array.append(line_array[-1])

        self.path_line = LineString(self.path_array)

        self.path_created = True

        self.past_theta = self.theta_array[0]

    
    def update_path(self, path_data):
        for pose in path_data.poses:
            point_x = pose.pose.position.x
            point_y = pose.pose.position.y
            point = (point_x, point_y)

            theta = compute_theta(pose)

            if point not in list(self.path_line.coords) and point not in self.path_done:
                self.path_line = LineString(self.path_line.coords[:] + [point]) 
                self.theta_array.append(theta)

    def path_callback(self, msg):
        path_data = msg.path
        self.state = msg.state
        self.linear_speed = msg.linear_speed
        self.angular_speed = msg.angular_speed

        if not self.path_created:
            self.create_pathLine(path_data)
            self.path_created = True

        else:
            self.update_path(path_data)


        if self.state == 1 and self.onetime_check:
            self.onetime_check = False
            self.status = 'done'
            self.stop()
            self.get_logger().info('Anomaly!! Going back to base...')
        else:

            if self.state == 3:
                self.linear_speed += 0.2  # speed up
            elif self.state == 4:
                self.linear_speed = self.linear_speed/2  # slow down
            elif self.state == 5:
                self.linear_speed = 0.0  # stop
                self.angular_speed = 0.0


    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_pub.publish(self.velocity)
            
def main(args=None):
    rclpy.init(args=args)
    node = FollowSpeed_controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
