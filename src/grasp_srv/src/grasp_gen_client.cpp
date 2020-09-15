#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "grasp_srv/GraspGen.h"
#include <cstdlib>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// msg
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "grasp_gen_client");

    ros::NodeHandle n;
    ros::ServiceClient client 
        = n.serviceClient<grasp_srv::GraspGen>("grasp_gen");
    
    grasp_srv::GraspGen srv;
    srv.request.object_poses.object_names.push_back("a_cups");
    srv.request.object_poses.object_names.push_back("a_cups");

    srv.request.object_poses.object_scales.push_back(0.5);
    srv.request.object_poses.object_scales.push_back(0.5);

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

    // object_clouds
    // Read from File
    std::string file_name = "/root/GraspService/src/grasp_srv/data/a_cups/mesh/cloud.pcd";
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile<pcl::PointNormal>(file_name, *cloud);
    // Transform this into a msg
    sensor_msgs::PointCloud2 test_point_cloud1;
    sensor_msgs::PointCloud2 test_point_cloud2;
    pcl::toROSMsg<pcl::PointNormal>(*cloud, test_point_cloud2);	
    srv.request.object_poses.object_point_clouds.push_back(test_point_cloud1);
    srv.request.object_poses.object_point_clouds.push_back(test_point_cloud2);

    if(client.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
        return 1;
    }
    return 0;
}
