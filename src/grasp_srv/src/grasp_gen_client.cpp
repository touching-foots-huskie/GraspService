#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "grasp_srv/GraspGen.h"
#include <cstdlib>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "grasp_gen_client");

    ros::NodeHandle n;
    ros::ServiceClient client 
        = n.serviceClient<grasp_srv::GraspGen>("grasp_gen");
    
    grasp_srv::GraspGen srv;
    srv.request.object_poses.object_names.push_back("a_cups");
    srv.request.object_poses.object_names.push_back("a_cups");

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 1;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    srv.request.object_poses.object_poses.push_back(pose);
    srv.request.object_poses.object_poses.push_back(pose);

    if(client.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
        return 1;
    }
    return 0;
}
