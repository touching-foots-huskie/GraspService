#include <ros/ros.h>

// Msg
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// Std
#include <iostream>
#include <sstream>

// JSON
#include "json_tools.h"


//global frame id
std::string scene_name = "1-1";


void SceneNameCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Choose Scene: [%s]", msg->data.c_str());
    scene_name = msg->data;
}


int main(int argc, char** argv)
{
    std::string workspace_path;
    workspace_path = std::string(argv[1]);

    std::string model_path;
    model_path = std::string(argv[2]);

    std::stringstream ss(argv[3]);
    bool pd_enable;
    ss >> std::boolalpha >> pd_enable;

    ros::init (argc, argv, "pub_scene");
    ros::NodeHandle nh;
    // Model Subscriber
    ros::Subscriber scene_sub  = nh.subscribe("scene_name", 1, SceneNameCallBack);
    // Point Cloud Publisher
    ros::Publisher scene_pointcloud_pub 
        = nh.advertise<sensor_msgs::PointCloud2> ("scene", 1);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_msg(new pcl::PointCloud<pcl::PointXYZRGBA>);

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        // Parse all pointclouds
        std::string json_filename = workspace_path + 
                                    scene_name + 
                                    "/object_data.json";
        std::ifstream json_file(json_filename);
        json object_datas;
        json_file >> object_datas;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merge(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for(auto i = 0; i < object_datas.size(); ++i) {
            std::string pcd_path;
            if(pd_enable) {
                pcd_path = workspace_path + 
                           scene_name +
                           "/" + std::to_string(i) + 
                           ".pcd";
            }
            else {
                std::string model_name = object_datas[i]["name"];
                pcd_path = model_path + model_name + 
                           "/visual_meshes/cloud.pcd";
            }
            
            // Load pointcloud
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_path, *cloud_in);
            // Transform data
            Eigen::Matrix4d transform;
            // Set Camera Pose
            if(pd_enable) {
                // Camera Pose Here
                transform << -1.271828999999999876e-01, 7.748804000000000247e-01, -6.191807999999999756e-01, 5.570000000000000506e-01,
                           9.885949000000000542e-01, 4.827069999999999972e-02, -1.426535999999999915e-01, 1.280000000000000027e-01,
                           -8.065110000000000334e-02, -6.302621000000000473e-01, -7.721820000000000350e-01, 5.879999999999999671e-01,
                           0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00;
            }
            else {
                matrix_parse(transform, object_datas[i]["pose_world"]);
            }
            
            Eigen::Affine3d T;
            T = transform;
            pcl::transformPointCloud<pcl::PointXYZRGBA>(
                *cloud_in,
                *cloud_out,
                T);
            // Assign a Random Color
            uint8_t r = static_cast<uint8_t>(((i+1) * 9999) % 255);
            uint8_t g = static_cast<uint8_t>(((i+1) * 8888) % 255);
            uint8_t b = static_cast<uint8_t>(((i+1) * 7777) % 255);
            uint8_t a = 255;
            for(auto &p: cloud_out->points) {
                p.r = r;
                p.g = g;
                p.b = b;
                p.a = a;
            }
            // Merge pd
            *merge += *cloud_out;
        }
        // print color
        sensor_msgs::PointCloud2 pd_msg;
        pcl::toROSMsg<pcl::PointXYZRGBA>(*merge, pd_msg);	
        pd_msg.header.frame_id = "world";  // Deal with base frame
        // pd_msg.header.stamp = ros::Time::now();
        scene_pointcloud_pub.publish(pd_msg);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}