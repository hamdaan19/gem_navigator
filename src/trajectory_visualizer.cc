#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/duration.h>

#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

ros::Publisher traj_vis_pub;
ros::Publisher act_path_pub;
ros::Publisher des_path_pub;

int desID = 0; // Desired
int actID = 0; // Actual

bool visualize = true;

bool isDesiredPathAvailable = false; 
nav_msgs::Path path;
nav_msgs::Path act_path;

void desiredTrajCallback(const nav_msgs::Path::ConstPtr &data)
{
    path = *data; 
    path.header.frame_id = "world"; 
    isDesiredPathAvailable = true; 
}

void actualTrajCallback(const nav_msgs::Odometry::ConstPtr &data)
{

    if (visualize)
    {
        visualization_msgs::Marker marker;

        geometry_msgs::PoseStamped this_pose; 

        act_path.header = data->header; 
        act_path.header.frame_id = "world"; 
        this_pose.header = data->header;

        this_pose.pose = data->pose.pose;
        act_path.poses.push_back(this_pose);

        act_path_pub.publish(act_path);

        // marker.header.frame_id = "world";
        // marker.header.stamp = ros::Time();
        // marker.ns = "Actual Trajectory";
        // marker.id = actID++;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.action = visualization_msgs::Marker::ADD;

        // marker.pose.position.x = data->pose.pose.position.x;
        // marker.pose.position.y = data->pose.pose.position.y;
        // marker.pose.position.z = data->pose.pose.position.z;

        // marker.pose.orientation.x = data->pose.pose.orientation.x;
        // marker.pose.orientation.y = data->pose.pose.orientation.y;
        // marker.pose.orientation.z = data->pose.pose.orientation.z;
        // marker.pose.orientation.w = data->pose.pose.orientation.w;

        // marker.lifetime = ros::Duration(20.0);

        // marker.scale.x = 0.025;
        // marker.scale.y = 0.025;
        // marker.scale.z = 0.025;

        // marker.color.a = 1;
        // marker.color.r = 1.0;
        // marker.color.g = 1.0;
        // marker.color.b = 1.0;

        // traj_vis_pub.publish(marker);
    }

    des_path_pub.publish(path); 
    // for (int i = 0; i < path.poses.size(); i++) {
    //     geometry_msgs::PoseStamped pose = path.poses[i];

    //     visualization_msgs::Marker des_marker;

    //     des_marker.header.frame_id = "world";
    //     des_marker.header.stamp = ros::Time();
    //     des_marker.ns = "Actual Trajectory";
    //     des_marker.id = desID++;
    //     des_marker.type = visualization_msgs::Marker::SPHERE;
    //     des_marker.action = visualization_msgs::Marker::ADD;

    //     des_marker.pose.position.x = pose.pose.position.x;
    //     des_marker.pose.position.y = pose.pose.position.y;
    //     des_marker.pose.position.z = pose.pose.position.z;

    //     des_marker.lifetime = ros::Duration(20.0);

    //     des_marker.scale.x = 0.025;
    //     des_marker.scale.y = 0.025;
    //     des_marker.scale.z = 0.025;

    //     des_marker.color.a = 1;
    //     des_marker.color.r = 0.0;
    //     des_marker.color.g = 1.0;
    //     des_marker.color.b = 0.0;

    //     traj_vis_pub.publish(des_marker);
    // }

}


int main(int argc, char **argv)
{
    std::string name = "trajectory_visualizer";
    ros::init(argc, argv, name);
    if (!ros::isInitialized())
    {
        ROS_WARN("Node is not initialized.");
        return 0;
    }

    // ROS Node Handle
    ros::NodeHandle nh;
    // Subscribers
    ros::Subscriber des_path_sub = nh.subscribe("/reference_path", 1000, desiredTrajCallback);
    ros::Subscriber act_traj_sub = nh.subscribe("/gem/base_footprint/odom", 1000, actualTrajCallback);
    // Pubslishers
    traj_vis_pub = nh.advertise<visualization_msgs::Marker>("trajectory_visualization", 1000);
    act_path_pub = nh.advertise<nav_msgs::Path>("actual_path", 1000);
    des_path_pub = nh.advertise<nav_msgs::Path>("desired_path", 1000);

    ros::spin();
}