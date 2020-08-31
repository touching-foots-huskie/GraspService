#include "ros/ros.h"
#include "grasp_srv/GraspGen.h"
#include <cstdlib>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "grasp_gen_client");

    ros::NodeHandle n;
    ros::ServiceClient client 
        = n.serviceClient<grasp_srv::GraspGen>("grasp_gen");
    
    grasp_srv::GraspGen srv;
    srv.request.model_name = "/home/harvey/NewProjects/gpd/tutorials/krylon.pcd";
    if(client.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
        return 1;
    }
    return 0;
}
