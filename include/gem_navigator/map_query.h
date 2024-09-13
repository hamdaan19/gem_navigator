#include "ros/ros.h"
#include "voxblox_ros/esdf_server.h"

#include "Eigen/Dense"

class MapQuery
{
public:
    MapQuery(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : 
    nh_(nh), nh_private_(nh_private), voxblox_server_(nh_, nh_private_) {

        double robot_radius = 0.1;
        voxblox_server_.setTraversabilityRadius(robot_radius);
        voxblox_server_.publishTraversable();
    }


    virtual ~MapQuery() {}
    double getMapDistance(const Eigen::Vector3d &position) const
    {
        if (!voxblox_server_.getEsdfMapPtr())
        {
            std::cout << "map does not exist!\n"; 
            return 0.0;
        }
        double distance = 0.0;
        if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, true, &distance))
        {
            std::cout << "unable to query map\n";
            return 0.0;
        }
        return distance;
    }

    void helloWorld() {
        std::cout << "Hello World!\n"; 
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Map!
    voxblox::EsdfServer voxblox_server_;
};