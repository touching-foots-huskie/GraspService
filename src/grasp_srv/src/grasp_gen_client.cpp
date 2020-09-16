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

// json related 
#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

// stream
#include <istream>
#include <fstream>

// Eigen related
#include <Eigen/Eigen>
#include <Eigen/Geometry>

void matrix_parse(Eigen::Matrix4d& matrix, const json& json_matrix) {
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            matrix(i, j) = json_matrix[i][j];
        }
    }
} 


int main(int argc, char **argv) 
{
    // ros init
    ros::init(argc, argv, "grasp_gen_client");
    ros::NodeHandle n;
    ros::ServiceClient client 
        = n.serviceClient<grasp_srv::GraspGen>("grasp_gen");
    
    // json path
    std::string scene_name = "1-1/";
    std::string data_path = "/home/harvey/Data/kinect_data/output/";
    std::string json_path = data_path + scene_name + "object_data.json";

    std::ifstream json_file("file.json");
    json object_datas;
    json_file >> object_datas;
    
    grasp_srv::GraspGen srv;

    // Go through all objects in scene
    for(auto id = 0; id < object_datas.size(); ++id) {
        // Parse name
        std::string name = object_datas[id]["name"];
        srv.request.object_poses.object_names.push_back(name);
        srv.request.object_poses.object_scales.push_back(1.0);
        // Parse pose
        Eigen::Matrix4d object_pose;
        matrix_parse(object_pose, object_datas[id]["pose_world"]);
        // Compute quat from matrix
        Eigen::Matrix3d q_matrix = object_pose.block(0, 0, 3, 3);
        Eigen::Quaterniond q(q_matrix);
        geometry_msgs::Pose pose;
        pose.position.x = object_pose(3, 0);
        pose.position.y = object_pose(3, 1);
        pose.position.z = object_pose(3, 2);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        srv.request.object_poses.object_poses.push_back(pose);
        // Load PointCloud
        std::string pd_filename = data_path + scene_name + std::to_string(id) + ".pcd";
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pd_filename, *cloud);
        // Transform this into a msg
        sensor_msgs::PointCloud2 point_cloud;
        pcl::toROSMsg<pcl::PointXYZRGBA>(*cloud, point_cloud);	
        srv.request.object_poses.object_point_clouds.push_back(point_cloud);
    }

    // call srv
    if(client.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
        return 1;
    }
    return 0;
}
