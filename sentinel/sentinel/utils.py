import PyKDL
import math

from math import dist, pi


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