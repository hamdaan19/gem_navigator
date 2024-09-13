#include <iostream>
#include <memory>
#include <vector> 

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"

#include "gem_navigator/QueryMap.h"

#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

std::shared_ptr<ros::Publisher> obs_point_pub_ptr;

std::vector<Eigen::Vector3d> obstacleLoc {
    Eigen::Vector3d(10,-3, 0),
    Eigen::Vector3d(22,-1, 0),
    Eigen::Vector3d(34,-3, 0),
    Eigen::Vector3d(-22,-3, 0),
    Eigen::Vector3d(-46,-1, 0),
};

bool query_map(gem_navigator::QueryMap::Request &req, gem_navigator::QueryMap::Response &res)
{
    Eigen::Vector3d position(req.x, req.y, req.z);
    
    double minDistance = 10000000; 
    for (Eigen::Vector3d obs : obstacleLoc) {
        double distance = (position - obs).norm(); 
        if (distance < minDistance) {
            minDistance = distance; 
        }
    }
    res.distance = minDistance; 
    return true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &data)
{
    double curr_x = data->pose.pose.position.x;
    double curr_y = data->pose.pose.position.y;
    double curr_z = data->pose.pose.position.z;

    Eigen::Vector3d curr_position(curr_x, curr_y, curr_z);

    double min_distance = (obstacleLoc[0] - curr_position).norm();
    int min_idx = 0; 
    for (int i = 1; i < obstacleLoc.size(); i++) {
        double dist = (obstacleLoc[i] - curr_position).norm();
        if (dist < min_distance) {
            min_distance = dist; 
            min_idx = i; 
        }
    }

    geometry_msgs::Point point;
    point.x = obstacleLoc[min_idx](0);
    point.y = obstacleLoc[min_idx](1);
    point.z = obstacleLoc[min_idx](2);

    obs_point_pub_ptr->publish(point); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "esdf_simulator_node");
    ros::NodeHandle nh;

    ros::ServiceServer map_query_service = nh.advertiseService("QuerySimulatedMapService", query_map);

    ros::Subscriber odom_sub = nh.subscribe("/gem/base_footprint/odom", 10, odom_callback);

    obs_point_pub_ptr = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::Point>("/closest_obstacle_location", 10));

    ros::spin();
}