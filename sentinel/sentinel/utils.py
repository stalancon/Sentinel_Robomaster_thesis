import PyKDL
import math
import rtree
import networkx as nx
import numpy as np
import rclpy.logging
import geometry_msgs

from shapely.geometry import LineString
from typing import Tuple, List, Union
from math import dist, pi
from geometry_msgs.msg import PoseStamped

Point = Union[np.ndarray, Tuple[float, float]]

def frame_from_pose(pose: geometry_msgs.msg.Pose) -> PyKDL.Frame:
    return PyKDL.Frame(
            PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
    )

def pose_from_frame(frame: PyKDL.Frame) -> geometry_msgs.msg.Pose:
    q = frame.M.GetQuaternion()
    p = frame.p
    return geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=p.x(), y=p.y(), z=p.z()),
            orientation=geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


def transform_from_state(x: float, y: float, theta: float):
    pos = PyKDL.Vector(x, y, 0.0)
    rot = PyKDL.Rotation.RotZ(theta)
    return PyKDL.Frame(V=pos, R=rot)


def transform_from_odom_msg(odom_msg):
    position_msg = odom_msg.pose.pose.position
    pos = PyKDL.Vector(position_msg.x, position_msg.y, position_msg.z)
    quaterion_msg = odom_msg.pose.pose.orientation
    rot = PyKDL.Rotation.Quaternion(quaterion_msg.x, quaterion_msg.y, quaterion_msg.z, quaterion_msg.w)
    return PyKDL.Frame(V=pos, R=rot)


def transform_msg(transform: PyKDL.Frame) -> geometry_msgs.msg.Transform:
    msg = geometry_msgs.msg.Transform()
    p = transform.p
    msg.translation = geometry_msgs.msg.Vector3(x=float(p[0]), y=float(p[1]), z=float(p[2]))
    q = transform.M.GetQuaternion()
    msg.rotation = geometry_msgs.msg.Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
    return msg



def distance_delta(past_pose, current_pose):
    current = [current_pose.pose.position.x, current_pose.pose.position.y]
    past = [past_pose.pose.position.x, past_pose.pose.position.y]

    delta_dist = dist(past, current)

    past_theta = compute_theta(past_pose)
    theta = compute_theta(current_pose)

    delta_theta = theta - past_theta

    return delta_dist, delta_theta


def compute_theta(pose):
    o = pose.pose.orientation
    orientation_gt = PyKDL.Rotation.Quaternion(o.x, o.y, o.z, o.w)
    theta, _, _ = orientation_gt.GetEulerZYX()

    theta = math.fmod(theta, 2*pi)

    return theta


def compute_graph(recorded_path, tolerance):
    index = rtree.index.Index()
    g = nx.Graph([(tuple(a), tuple(b), {'weight': float(np.linalg.norm(np.asarray(a) - np.asarray(b)))}) 
                  for a, b in zip(recorded_path, recorded_path[1:])])
    for i, (x, y) in enumerate(recorded_path):
        index.insert(i, (x, y, x, y), obj=(x, y)) 
    for point in recorded_path:
        connect_point(g, index, point, tolerance)                
    return g, index

def connect_point(g, index, point, tolerance):
    p = tuple(point)
    roi = (point[0] - tolerance, point[1] - tolerance, point[0] + tolerance, point[1] + tolerance)
    for o_point in (o.object for o in index.intersection(roi, objects=True) if o.object != p):
        g.add_edge(p, o_point, weight=float(np.linalg.norm(point - o_point)))     

def find_path(recorded_path, source, target, tolerance):
    g, index = compute_graph(recorded_path, tolerance)
    source = tuple(source)
    target = tuple(target)
    if source not in g:
        connect_point(g, index, source, tolerance)
    if target not in g:
        connect_point(g, index, target, tolerance)       
    return nx.shortest_path(g, source=source, target=target, weight='weigth')


def construct_path(path):
        pose_array = []
        # logger = rclpy.logging.get_logger("my")

        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            next_pose = PoseStamped()
            next_pose.pose.position.x = x0
            next_pose.pose.position.y = y0

            theta = math.atan2(y1 - y0, x1 - x0)
 
            next_pose.pose.orientation.w = np.cos(theta/2)
            next_pose.pose.orientation.z = np.sin(theta/2)

            pose_array.append(next_pose)

            # logger.info(f'x0 {x0}, y0 {y0} , theta {theta},  x1 {x1}, y1 {y1}, next pose: {next_pose}')

        return pose_array


def create_line_object(path):
        poses = path.poses
        
        line_array = []

        for p in poses:
            point = (p.pose.position.x, p.pose.position.y)
            line_array.append(point)

        path_line = LineString(line_array)

        return path_line