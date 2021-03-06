
import numpy as np
import rclpy
import geometry_msgs
import PyKDL
import tf2_ros

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from sentinel.utils import  frame_from_pose, transform_from_state, pose_from_frame, transform_msg, transform_from_odom_msg

def wiener_update(value, tau, time_step):
    return np.exp(-time_step / tau) * value + np.sqrt(2 * time_step / tau) * np.random.normal(0, 1)

class StateEstimation:

    def __init__(self, *, L, epsilon):
        self.L = L
        self._error = 0.0
        self.epsilon = epsilon
        self._gt = None

    def update(self, gt):
        if self._gt is not None:
            delta = abs(gt - self._gt)
        else:
            delta = 0.0
        self._gt = gt
        self._error =  wiener_update(self._error, self.L, delta)
        return gt + self.epsilon * self._error

 
class NoiseEstimator(Node):

    def __init__(self):
        super().__init__('noise_estimation')

        # exact pose subscriber
        self.create_subscription(PoseStamped,'exact_pose', self.pose_callback, 10)
        self.pose = Pose()

        # odometry subscriber
        self.create_subscription(Odometry,'odom', self.odom_callback, 10)

        # pose publisher
        self.pose_publisher = self.create_publisher(PoseStamped,'noisy_pose', 10)

        # Tf broadcaster
        self.tf_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)

        # initialize estimation
        self.x = StateEstimation(L=1, epsilon=0.1)
        self.y = StateEstimation(L=1, epsilon=0.1)
        self.theta = StateEstimation(L=0.5, epsilon=0.1)

        self.odom_msg = Odometry()

    def broadcast_tf(self, transform: PyKDL.Frame, parent: str, child: str):
        msg = geometry_msgs.msg.TransformStamped(transform=transform_msg(transform))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        self.tf_broadcaster.sendTransform(msg)

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_msg.header.frame_id = msg.header.frame_id

    def pose_callback(self, exact_pose):

        final_estimation = PoseStamped()
        final_estimation.header.frame_id = exact_pose.header.frame_id
        o = exact_pose.pose.orientation
        # theta_gt = 2 * np.arctan2(exact_pose.pose.orientation.z, exact_pose.pose.orientation.w)

        orientation_gt = PyKDL.Rotation.Quaternion(o.x, o.y, o.z, o.w)
        theta_gt, _, _ = orientation_gt.GetEulerZYX()


        x_e = self.x.update(exact_pose.pose.position.x)
        y_e = self.y.update(exact_pose.pose.position.y)
        theta_e = self.theta.update(theta_gt)

        final_estimation.pose.position.x = x_e
        final_estimation.pose.position.y = y_e
        final_estimation.pose.orientation.w = np.cos(theta_e/2)
        final_estimation.pose.orientation.z = np.sin(theta_e/2)

        transform = transform_from_odom_msg(self.odom_msg).Inverse() * transform_from_state(x_e, y_e, theta_e)
        self.broadcast_tf(transform, exact_pose.header.frame_id, self.odom_msg.header.frame_id)

        self.pose_publisher.publish(final_estimation)

def main(args=None):

    rclpy.init(args=args)
    node = NoiseEstimator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # node.destroy_node()
    rclpy.shutdown()
