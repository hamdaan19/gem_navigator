#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

std::string child_frame;

void callback(const nav_msgs::OdometryConstPtr &data)
{
    double x = data->pose.pose.position.x;
    double y = data->pose.pose.position.y;
    double z = data->pose.pose.position.z;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q;
    q.setX(data->pose.pose.orientation.x);
    q.setY(data->pose.pose.orientation.y);
    q.setZ(data->pose.pose.orientation.z);
    q.setW(data->pose.pose.orientation.w);

    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", child_frame));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    
    if (argc != 2) {
        ROS_ERROR("need turtle name as argument"); 
        return -1;
    }
    child_frame = argv[1];

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gem/base_footprint/odom", 10, &callback);

    ros::spin();
    return 0;
}