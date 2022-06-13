

import rclpy
import numpy as np
from math import pi

from rclpy.node import Node
from shapely.geometry import LineString, Point
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sentinel_msgs.msg import PathMsg
from sentinel.utils import compute_theta, frame_from_pose, transform_from_state, pose_from_frame

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

        # self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose_sentinel',1)
        # self.gp = PoseStamped()

        self.status = 'start'

        self.onetime_check = True
        self.path_created = False
        self.path_check = True
        self.second_path = False


    def pose_callback(self, noisy_pose):

        if self.path_created:

            if not self.status == 'done':

                self.gp.header.frame_id = noisy_pose.header.frame_id
                pose_point = Point(noisy_pose.pose.position.x, noisy_pose.pose.position.y)

                # Get the point in the line that's closest to the current pose
                s = self.path_line.project(pose_point)
                goal_pose = self.path_line.interpolate(s)
                
                self.gp.pose.position.x = goal_pose.x
                self.gp.pose.position.y = goal_pose.y

                # Get distance between the current position and the goal pose
                dist_goal = goal_pose.distance(pose_point)

                if goal_pose == self.end_point:
                    dist_goal = self.end_zone.distance(pose_point)


                # Check if the path is done
                if goal_pose == self.end_point and dist_goal < 0.2:

                        self.status = 'done'
                        self.stop()

                        if self.state == 2:
                            self.get_logger().info('Back at base')
                        else:
                            self.get_logger().info('End of path')
                else:

                    current_theta = compute_theta(noisy_pose)

                    target_theta = goal_pose.z

                    # obtain the target pose with respect to the current robot frame
                    current_pose = frame_from_pose(noisy_pose.pose)
                    target_pose = transform_from_state(goal_pose.x, goal_pose.y, target_theta)
                    next_pose = pose_from_frame(current_pose.Inverse() * target_pose)

                    # Vector
                    vector = [next_pose.position.x, next_pose.position.y]

                    self.gp.pose.orientation.w = np.cos(target_theta/2)
                    self.gp.pose.orientation.z = np.sin(target_theta/2)
                    self.goal_pose_pub.publish(self.gp)
                    
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

        self.past_theta = self.theta_array[0]
        self.end_point = Point(self.path_line.coords[-1])

        # Add buffer zone to the location of the follower
        self.end_zone = Point(line_array[-1]).buffer(0.2)


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
            self.get_logger().info('Path created!')
 
        if self.state == 1 and self.onetime_check:
            self.onetime_check = False
            self.status = 'done'
            self.stop()
            self.get_logger().info('Anomaly!!')
            self.path_created = False
            self.second_path = True

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
        self.velocity.linear.y = 0.0
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
