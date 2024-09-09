#!/usr/bin/env python3
import math
import rospy
import rospkg
import numpy as np
import csv
import sys

from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry

import os

import time

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("gem_navigator"))

from src.callbacks import Callback

cb = Callback()

def get_waypoints(filename):

    dirname = rp.get_path("gem_navigator")
    file = dirname + filename

    with open(file) as f:
        path_points = [list(line) for line in csv.reader(f)]

    way_pts = np.array([path_points[0][0], path_points[0][1], path_points[0][2]])
    for i in range(1, len(path_points), 10):
        pt = np.array(path_points[i][:3])
        way_pts = np.vstack([way_pts, pt])

    return way_pts

def straight_line_waypoints(pos_x=0.0, pos_y=0.0, pos_z=0.0): 
    
    way_pts = np.array([[pos_x, pos_y, pos_z]])
    for i in np.arange(0.05, 1+0.05, 0.05): 
        pt = (1-i)*np.array([[pos_x, pos_y, pos_z]]) + i*np.array([[pos_x+100, pos_y, pos_z]])
        way_pts = np.vstack([way_pts, pt])

    print(way_pts)

    return way_pts

def circular_waypoints(): 
    way_pts = np.array([[0, 0, 0]])

    n_pts = 16
    radius = 25
    step = 2*math.pi/n_pts

    center_x = 0
    center_y = radius

    for theta in np.arange(-0.5*math.pi+step, 0.5*math.pi, step):

        x = center_x + radius*math.cos(theta)
        y = center_y + radius*math.sin(theta)

        wp = np.array([[x, y, 0.0]])
        way_pts = np.vstack([way_pts, wp])

    print(way_pts)
    return way_pts

    
if __name__ == "__main__": 

    rospy.init_node("path_publisher_node", anonymous=False)

    state_sub = rospy.Subscriber("/gem/base_footprint/odom", Odometry, cb.state_callback, queue_size=10)

    reference_path_pub = rospy.Publisher("/path_waypoints", PoseArray, queue_size=5)

    # way_pts = np.array([
    #     [0.0, 0.0, 0.0],
    #     [2.5, 0.0, 0.0],
    #     [5.0, 0.0, 0.0],
    #     [7.5, 0.0, 0.0],
    #     [10.0, 0.0, 0.0],
    #     [12.5, 0.0, 0.0],
    #     [15.000000000000002, 0.0, 0.0],
    #     [17.5, 0.0, 0.0],
    #     [20.0, 0.0, 0.0],
    #     [22.5, 0.0, 0.0],
    #     [25.0, 0.0, 0.0],
    #     [27.500000000000004, 0.0, 0.0],
    #     [30.000000000000004, 0.0, 0.0],
    #     [32.5, 0.0, 0.0],
    #     [35.0, 0.1, 0.0],
    #     [37.5, 0.1, 0.0],
    #     [40.0, 0.1, 0.0],
    #     [42.5, 0.1, 0.0],
    #     [45.0, 0.1, 0.0],
    #     [47.5, 0.1, 0.0],
    #     [50.0, 0.1, 0.0]
    # ])

    # way_pts = np.array([
    #     [0,0,0],
    #     [50,0,0],
    #     [100,11.5,0],
    #     [120,25,0],
    #     [140,50,0],
    #     [147.5,75,0],
    #     [150,100,0]
    # ])

    # way_pts = straight_line_waypoints('/waypoints/wps.csv')
    way_pts = get_waypoints('/waypoints/wps.csv')
    # way_pts = circular_waypoints()

    array_msg = PoseArray()
    array_msg.header.stamp = rospy.Time.now()
    for i in range(way_pts.shape[0]):
        
        pt = way_pts[i]

        pose_msg = Pose()
        pose_msg.position.x = float(pt[0])
        pose_msg.position.y = float(pt[1])
        pose_msg.position.z = float(pt[2])

        array_msg.poses.append(pose_msg)

    print("I am here!")

    while(not rospy.is_shutdown()):
        time.sleep(0.1)
        reference_path_pub.publish(array_msg)
        break

    print(way_pts.shape[0])

    print("Path message published!")
        