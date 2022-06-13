

import rclpy
import numpy as np
import math 

from math import pi
from rclpy.node import Node
from shapely.geometry import LineString, Point
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sentinel_msgs.msg import PathMsg
from sentinel.utils import compute_theta,  frame_from_pose, transform_from_state, pose_from_frame

class FollowSpeed_controller(Node):

    def __init__(self):
        super().__init__('followspeed_controller')
        
        # path subscriber
        self.create_subscription(PathMsg, 'follow_trajectory', self.path_callback, 10)

        # noisy pose subscriber
        self.create_subscription(PoseStamped,'noisy_pose', self.pose_callback, 10)
        self.pose = Pose()

        # self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose_follower',1)
        # self.gp = PoseStamped()

        # velocity publisher
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.velocity = Twist()

        self.path_created = False
        self.onetime_check = True

        self.status = 'start'
        self.counter = 0


    def pose_callback(self, noisy_pose):

        if self.path_created:

            if not self.status == 'done':
                self.gp.header.frame_id = noisy_pose.header.frame_id

                pose_point = Point(noisy_pose.pose.position.x, noisy_pose.pose.position.y)

                # Get the point in the line that's closest to the current pose
                goal_pose = self.path_line.interpolate(self.path_line.project(pose_point))

                target_theta = goal_pose.z
                current_theta = compute_theta(noisy_pose)
                
                # obtain the target pose with respect to the current robot frame
                current_pose = frame_from_pose(noisy_pose.pose)
                target_pose = transform_from_state(goal_pose.x, goal_pose.y, target_theta)
                next_pose = pose_from_frame(current_pose.Inverse() * target_pose)

                # Unit vector
                vector = [next_pose.position.x, next_pose.position.y]
                
                theta = target_theta - current_theta

                theta = np.fmod(theta,2*pi)

                if theta > pi:
                    theta -= 2*pi
                elif theta < -pi:
                    theta += 2*pi

                self.velocity.angular.z = self.angular_speed * theta

                k = 1
                self.velocity.linear.x = k * vector[0] + self.linear_speed * np.cos(theta)
                self.velocity.linear.y = k * vector[1] + self.linear_speed * np.sin(theta)

                speed = np.sqrt(self.velocity.linear.x ** 2 + self.velocity.linear.y ** 2)
                if speed > self.linear_speed:
                    self.velocity.linear.x *= self.linear_speed/speed
                    self.velocity.linear.y *= self.linear_speed/speed

                self.velocity_pub.publish(self.velocity)

                # self.gp.pose.position.x = goal_pose.x
                # self.gp.pose.position.y = goal_pose.y
                # self.gp.pose.orientation.w = np.cos(target_theta/2)
                # self.gp.pose.orientation.z = np.sin(target_theta/2)
                # self.goal_pose_pub.publish(self.gp)


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
        
        self.theta_array.append(self.theta_array[-1])
        self.theta_array = np.unwrap(self.theta_array)

        self.path_array = line_array
        self.path_array.append(line_array[-1])

        self.path_line = LineString([(x, y, t) for (x, y), t in zip(self.path_array, self.theta_array)])

        self.path_created = True

        self.past_theta = self.theta_array[0]

    
    def update_path(self, path_data):
        for pose in path_data.poses:
            point_x = pose.pose.position.x
            point_y = pose.pose.position.y

            theta = compute_theta(pose)

            point = (point_x, point_y, theta)

            if point not in list(self.path_line.coords):
                self.path_line = LineString(self.path_line.coords[:] + [point]) 

    def path_callback(self, msg):
        path_data = msg.path
        self.state = msg.state
        self.linear_speed = msg.linear_speed
        self.angular_speed = msg.angular_speed

        if not self.path_created:
            self.create_pathLine(path_data)
            self.path_created = True
            self.status = 'start'
            self.onetime_check = True

        else:
            self.update_path(path_data)


        if self.state == 1 and self.onetime_check:
            self.onetime_check = False
            self.status = 'done'
            self.stop()
            self.get_logger().info('Anomaly!!')
            self.path_created = False
            self.path_done =  []
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
