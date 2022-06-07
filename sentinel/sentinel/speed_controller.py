

import rclpy
import numpy as np
import math

from math import dist, pi
from rclpy.node import Node
from shapely.geometry import LineString, Point
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sentinel_msgs.msg import PathMsg
from sentinel.utils import compute_theta


class Speed_controller(Node):

    def __init__(self):
        super().__init__('speed_controller')
        
        # path subscriber
        self.create_subscription(PathMsg, 'path_trajectory', self.path_callback, 10)

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 10)
        self.pose = Pose()

        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        self.status = 'start'

        self.onetime_check = True
        self.path_created = False
        self.path_check = True

        self.last_dist = 20


    def pose_callback(self, noisy_pose):

        if self.path_created:

            if not self.status == 'done':

                pose_point = Point(noisy_pose.pose.position.x, noisy_pose.pose.position.y)

                # Get the point in the line that's closest to the current pose
                goal_pose = self.path_line.interpolate(self.path_line.project(pose_point))

                # Get distance between the current position and the goal pose
                dist_goal = goal_pose.distance(pose_point)

                if self.state == 2 and self.path_check:
                    for i, p in enumerate(list(self.path_line.coords)):
                        if i == 0 and Point(p) != goal_pose:
                            goal_pose = Point(self.path_line.coords[i+1])
                            self.path_line = LineString(self.path_line.coords[i+1:]) 
                            self.past_theta = self.theta_array[0]
                            self.theta_array = self.theta_array[i+1:]
                            dist_goal = goal_pose.distance(pose_point)
                    self.path_check = False

                if goal_pose == self.end_point:
                    dist_goal = self.end_zone.distance(pose_point)
                    
                    if dist_goal > self.last_dist:
                        dist_goal = 0.2
                    else:
                        self.last_dist = dist_goal

                # Check if the path is done
                if goal_pose == self.end_point and dist_goal < 0.3:

                        self.status = 'done'
                        self.stop()

                        if self.state == 2:
                            self.get_logger().info('Back at base')
                        else:
                            self.get_logger().info('End of path')
                else:
                    
                    if dist_goal < 0.2:
                        for i, p in enumerate(list(self.path_line.coords)):
                            if i == 0 and Point(p) == goal_pose:
                                self.path_line = LineString(self.path_line.coords[i+1:]) 
                                self.past_theta = self.theta_array[0]
                                self.theta_array = self.theta_array[i+1:]  
                                
                    for i, p in enumerate(list(self.path_line.coords)):
                        if i == 0 and Point(p) != goal_pose:
                            goal_pose = Point(self.path_line.coords[i+1])
                            self.path_line = LineString(self.path_line.coords[i+1:]) 
                            self.past_theta = self.theta_array[0]
                            self.theta_array = self.theta_array[i+1:]  

                    # Unit vector
                    vector = [goal_pose.x - pose_point.x, goal_pose.y - pose_point.y]
                    unit_vector = vector / np.linalg.norm(vector)

                    current_theta = compute_theta(noisy_pose)

                    target_theta = self.theta_array[0]

                    side = self.past_theta - target_theta

                    if self.state == 2:

                        if target_theta >= 0.0 and current_theta < 0.0:
                            target_theta = -target_theta
                        if target_theta <= 0.0 and current_theta > 0.0:
                            target_theta = abs(target_theta)

                    theta = target_theta - current_theta
                    
                    if side == 0.0: # check if i need to turn

                        if abs(theta) > 0.15:
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
                    
                    # if self.state == 2:
                    #     self.get_logger().info(f'gp {goal_pose}')
                    #     self.get_logger().info(f'c pose {pose_point}')
                    #     self.get_logger().info(f't thet {target_theta}')
                    #     self.get_logger().info(f'c theta {current_theta}')
                    #     self.get_logger().info(f't {theta}')
                    
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

            if self.state == 2:
                theta = compute_theta(pose)
            else:
                theta =  2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                
            self.theta_array.append(theta)

        self.path_array = line_array
        self.path_array.append(line_array[-1])

        self.path_line = LineString(self.path_array)

        self.past_theta = self.theta_array[0]
        self.end_point = Point(line_array[-1])

        # Add buffer zone to the location of the follower
        self.end_zone = Point(line_array[-1]).buffer(0.1)

        if self.state == 2:
            self.get_logger().info(f'path {self.path_line}')
            self.get_logger().info(f'thetas {self.theta_array}')

    def path_callback(self, path_msg):
        path_data = path_msg.path
        self.state = path_msg.state
        self.linear_speed = path_msg.linear_speed
        self.angular_speed = path_msg.angular_speed


        if not self.path_created:

            self.create_pathLine(path_data)
            self.status = 'start'
            self.path_created = True
            self.onetime_check = True
            self.path_check = True
 
        if self.state == 1 and self.onetime_check:
            self.onetime_check = False
            self.status = 'done'
            self.stop()
            self.get_logger().info('Anomaly!!')
            self.path_created = False

        else:

            if self.state == 4:
                self.linear_speed = self.linear_speed/2  # slow down
                self.angular_speed -= 0.1
            elif self.state == 3:
                self.linear_speed += 0.2  # speed up
                self.angular_speed += 0.1
            elif self.state == 5:
                self.linear_speed = 0.0  # stop
                self.angular_speed = 0.0
            

    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.velocity_pub.publish(self.velocity)
            
def main(args=None):
    rclpy.init(args=args)
    node = Speed_controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
