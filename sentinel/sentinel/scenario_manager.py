
import rclpy
import numpy as np

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Scenario_manager(Node):

    def __init__(self):
        super().__init__('scenario_manager')

        # Parameter to chose experimental scenario
        self.path_type = self.declare_parameter('path_type', 'simple').value

        # Simulation parameter
        sim = self.declare_parameter('sim', False).value

        self.path_to_follow = Path()
        poses_path = Path()

        # Path publisher
        self.path_publisher = self.create_publisher(Path,'/path', 10)
        
        if self.path_type == 'complex':

            path_1 = Path()
            path_2 = Path()

            path1_poses = [[2.5, 0.0, 0.0], [3.8, 0.0, 0.0], [4.2, 0.0, 0.0], [4.9, 1.0, 1.6], [4.9, 1.5, 1.6], [4.9, 2.0, 1.6], [4.9, 2.3, 1.6],
                 [5.7, 2.6, 0.0], [6.3, 2.6, 0.0], [6.5, 2.6, 0.0], [7.0, 2.6, 0.0],[7.5, 2.6, 0.0],[8.0, 2.6, 0.0], [8.5, 2.6, 0.0],[8.7, 2.6, 0.0], [9.5, 2.6, 0.0], [10.5, 3.9, 1.5], [10.5, 7.5, 1.5]]
            path2_poses = [[2.5, 0.0, 0.0], [3.8, 0.0, 0.0], [4.2, 0.0, 0.0], [4.9, -1.0, -1.6], [4.9, -1.3, -1.6], [4.9, -1.7, -1.6], [4.9, -2.3, 0.0], 
                 [5.5, -2.3, 0.0], [6.0, -2.3, 0.0], [7.0, -2.4, 0.0], [8.3, -2.4, 0.0], [9.3, -2.4, 0.0], [10.0, -2.4, 0.0], [10.5, -1.7, 1.6], [10.5, -0.6, 1.6], [10.5, 2.9, 1.6],  [10.5, 7.5, 1.6]]
            
            paths = [path1_poses, path2_poses]

            for path in paths:
                if path == path1_poses:
                    path_1.poses = self.create_pose_path(path)
                else:
                    path_2.poses = self.create_pose_path(path)

            # Chose the shortest path
            if len(path_1.poses) < len(path_2.poses):
                self.path_to_follow = path_1
                self.backup_path = path_2
            else:
                self.path_to_follow = path_2
                self.backup_path = path_1


        elif self.path_type == 'simple':

            if sim:
                poses_path = [[1.0, -1.5, 0.0], [1.3, -1.5, 0.0], [1.9, -0.8, 1.5], [1.9, -0.7, 1.5], [1.9, -0.6, 1.5], [1.9, 1.0, 1.5], [1.9, 1.8, 1.5]] 
            else:
                # poses_path = [[1.0, 0.0, 0.0], [1.5, 0.0, 0.0], [2.0, -0.8, 1.5], [2.0, -0.7, 1.5], [2.0, -0.6, 1.5], [2.0, 1.0, 1.5],[2.0, 1.8, 1.5], [2.0, 2.0, 1.5]] 
                poses_path = [[1.0, 0.0, 0.0], [1.5, 0.0, 0.0], [2.0, -0.8, -1.6], [2.0, -0.7, -1.6], [2.0, -0.3, -1.6], [2.0, -1.0, -1.6], [2.0, -1.8, -1.6],[2.0, -2.0, -1.6],[2.0, -2.2, -1.6]]
            
            self.path_to_follow.poses = self.create_pose_path(poses_path)



        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def create_pose_path(self, path):
        pose_array = []
        next_pose = PoseStamped()
        next_pose.header.frame_id = 'world'

        for i in path:
                next_pose = PoseStamped()
                next_pose.pose.position.x = i[0]
                next_pose.pose.position.y = i[1]
                next_pose.pose.orientation.w = np.cos(i[2]/2)
                next_pose.pose.orientation.z = np.sin(i[2]/2)
                pose_array.append(next_pose)

        return pose_array


    def timer_callback(self):

        if self.path_type == 'complex':
            self.path_publisher.publish(self.path_to_follow)
            self.path_publisher.publish(self.backup_path)
        else:
            self.path_publisher.publish(self.path_to_follow)


def main(args=None):

    rclpy.init(args=args)
    node = Scenario_manager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
