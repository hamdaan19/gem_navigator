#include <iostream>
#include <memory>

#include "ros/ros.h"
#include "gem_navigator/map_query.h"
#include "gem_navigator/QueryMap.h"

std::shared_ptr<MapQuery> map_ptr;

bool query_map(gem_navigator::QueryMap::Request &req, gem_navigator::QueryMap::Response &res)
{
    Eigen::Vector3d position(req.x, req.y, req.z);
    // node.getEsdfMapPtr()->getDistanceAtPosition(position, &distance);
    map_ptr->helloWorld(); 
    res.distance = map_ptr->getMapDistance(position);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_distance_query_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // MapQuery map(nh, private_nh);

    map_ptr = std::make_shared<MapQuery>(nh, private_nh);

    ros::ServiceServer map_query_service = nh.advertiseService("QueryMapService", query_map);

    // voxblox::EsdfServer node(nh, private_nh);

    // node.setTraversabilityRadius(1.0);
    // node.publishTraversable();

    // map.getMapDistance(Eigen::Vector3d(10, 0, 1)); 

    // while(ros::ok()){
    //     // node.getEsdfMapPtr()->getDistanceAtPosition(position, &distance);
    //     // std::cout << "Distance: " << distance << "\n"; 
    //     loop_rate.sleep();
    // }

    ros::spin();
}