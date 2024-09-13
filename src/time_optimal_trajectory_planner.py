#!/usr/bin/env python3
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time
import math

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry

from tf.transformations import quaternion_from_euler
import rospkg 
import sys

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("gem_navigator"))

from src.callbacks import Callback

cb = Callback()

# Global Parameters
time_step = rospy.get_param("/gem_navigator/t_step")

def plan_trajectory(way_pts):

    ta.setup_logging("INFO")

    ################################################################################
    # PATH SCALARS
    i = 0
    N = way_pts.shape[0]
    print("Number of Waypoints Given: {}".format(N))
    total_chord_length = 0 
    # Computing the chord length
    while (i < N-1): 
        dist = np.linalg.norm( way_pts[i] - way_pts[i+1] )
        total_chord_length += dist
        i += 1
    i = 0
    p = 0
    p_scalars = []
    p_scalars.append(0.0)
    # Computing path scalars using the chord length method
    while (i < N-1):
        dist = np.linalg.norm( way_pts[i] - way_pts[i+1] )
        p += (dist/total_chord_length)
        p_scalars.append( p )
        i += 1

    ################################################################################
    vel_limits, accel_limits = np.array([2.5, 2.5]), np.r_[1.5,1.5]
    path_scalars = np.asarray(p_scalars)

    path = ta.SplineInterpolator(path_scalars, way_pts, bc_type="clamped")

    ################################################################################
    # Create velocity bounds, then velocity constraint object
    vlim = np.vstack((-vel_limits, vel_limits)).T
    # Create acceleration bounds, then acceleration constraint object
    alim = np.vstack((-accel_limits, accel_limits)).T
    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(
        alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

    # Setup a parametrization instance. The keyword arguments are
    # optional.
    instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')
    parameterization = instance.compute_parameterization(0, 0, True)

    jnt_traj = instance.compute_trajectory(0, 0)

    # ################################################################################
    duration = jnt_traj.duration
    print("Found optimal trajectory with duration {:f} sec".format(duration))

    ################################################################################
    # The output trajectory is an instance of
    # :class:`toppra.interpolator.AbstractGeometricPath`.
    n_samples = 100
    ts_sample = np.linspace(0, jnt_traj.duration, n_samples)
    qs_sample = jnt_traj(ts_sample)
    qds_sample = jnt_traj(ts_sample, 1)
    qdds_sample = jnt_traj(ts_sample, 2)

    # time_step = 0.5

    robot_path = Path()
    robot_path.header.stamp = rospy.Time.now()

    for t in np.arange(0, jnt_traj.duration-time_step, time_step): 

        position_t0 = jnt_traj(t)
        position_t1 = jnt_traj(t+time_step)

        delta_y = float(position_t1[1] - position_t0[1])
        delta_x = float(position_t1[0] - position_t0[0])

        yaw = math.atan(delta_y/delta_x)               # yaw angle in radians 
        if (delta_y >= 0 and delta_x < 0):
            yaw = yaw + math.pi
        elif (delta_y < 0 and delta_x < 0):
            yaw = yaw - math.pi

        quat = quaternion_from_euler(0.0, 0.0, yaw)    # roll, pitch, yaw

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = float(position_t0[0])
        pose_msg.pose.position.y = float(position_t0[1])
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.w = quat[3]

        robot_path.poses.append(pose_msg)

    reference_path_pub.publish(robot_path)


def callback(data): 

    poses = data.poses
    n_poses = len(data.poses)

    waypoints = np.zeros(shape=(n_poses, 2))
  
    for i in range(n_poses): 
        # waypoints[i] = [poses[i].position.x + cb.state_position_x, poses[i].position.y + cb.state_position_y]
        waypoints[i] = [poses[i].position.x, poses[i].position.y]

    plan_trajectory(waypoints)



if __name__ == "__main__": 
    rospy.init_node("trajectory_planner_node", anonymous=False)

    waypoints_sub = rospy.Subscriber("/path_waypoints", PoseArray, callback, queue_size=10)
    state_sub = rospy.Subscriber("/gem/base_footprint/odom", Odometry, cb.state_callback, queue_size=10)
    reference_path_pub = rospy.Publisher("/reference_path", Path, queue_size=5)

    rospy.spin()
