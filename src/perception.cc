#include <iostream>

#include "ros/ros.h"

#include "Eigen/Dense"

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"

void filter_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cld_input, pcl::PointCloud<pcl::PointXYZ>::Ptr cld_filtered)
{
    // Creating a pass through filter to remove points that correspond to the ground
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(cld_input);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(-1.9, 15.0);
    passthrough.filter(*cld_filtered);

}

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &data)
{
    std::cout << "h: " << data->height << " w: " << data->width << std::endl;
    std::cout << "Waypoint received!\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cld_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    filter_cloud(data, cld_filtered);

    std::cout << "filtered h: " << cld_filtered->height << " filtered w: " << cld_filtered->width << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "Hello World\n";

    ros::init(argc, argv, "submap_node");
    ros::NodeHandle nh;

    ros::Subscriber velodyne_sub = nh.subscribe("/gem/velodyne_points", 10, callback);

    ros::spin();
}