#!/usr/bin/env python3
import math
import rospy
import numpy as np

from geometry_msgs.msg import PoseArray, Pose

if __name__ == "__main__": 

    rospy.init_node("path_publisher_node", anonymous=False)

    reference_path_pub = rospy.Publisher("/path_waypoints", PoseArray, queue_size=5)

    way_pts = np.array([
        [0.0, 0.0, 0.0],
        [2.5, 0.0, 0.0],
        [5.0, 0.0, 0.0],
        [7.500000000000001, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [12.5, 0.0, 0.0],
        [15.000000000000002, 0.0, 0.0],
        [17.5, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [22.5, 0.0, 0.0],
        [25.0, 0.0, 0.0],
        [27.500000000000004, 0.0, 0.0],
        [30.000000000000004, 0.0, 0.0],
        [32.5, 0.0, 0.0],
        [35.0, 0.1, 0.0],
        [37.5, 0.1, 0.0],
        [40.0, 0.1, 0.0],
        [42.5, 0.1, 0.0],
        [45.0, 0.1, 0.0],
        [47.5, 0.1, 0.0],
        [50.0, 0.1, 0.0]
    ])

    array_msg = PoseArray()
    array_msg.header.stamp = rospy.Time.now()
    for i in range(way_pts.shape[0]):
        
        pt = way_pts[i]

        pose_msg = Pose()
        pose_msg.position.x = float(pt[0])
        pose_msg.position.y = float(pt[1])
        pose_msg.position.z = float(pt[2])

        array_msg.poses.append(pose_msg)

    rate = rospy.Rate(10)
    while(not rospy.is_shutdown()) :
        rate.sleep()
        reference_path_pub.publish(array_msg)
        break

    print(way_pts.shape[0])

    print("Path message published!")
        