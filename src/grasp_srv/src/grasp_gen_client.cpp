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
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include "grasp_srv/Grasps.h"

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

// Boost
#include <boost/bind.hpp>
#include <boost/function.hpp>

// JSON
#include "json_tools.h"


// global data
grasp_srv::Grasps grasps;
std::string model_name;
int grasp_id;


/*
When Receive a scene name, ask for grasp pose in the scene
*/
void SceneNameCallBack(const std_msgs::String::ConstPtr& msg, 
                       ros::NodeHandle n, const std::string& data_path) {
    ROS_INFO("Choose Scene: [%s]", msg->data.c_str());
    std::string scene_name = msg->data;
    std::string json_path = data_path + scene_name + "/object_data.json";
    std::cout << json_path << std::endl;
    std::ifstream json_file(json_path);
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
        std::string pd_filename = data_path + scene_name + "/" + std::to_string(id) + ".pcd";
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pd_filename, *cloud);
        // Transform this into a msg
        sensor_msgs::PointCloud2 point_cloud;
        pcl::toROSMsg<pcl::PointXYZRGBA>(*cloud, point_cloud);	
        srv.request.object_poses.object_point_clouds.push_back(point_cloud);
    }
    // Start Service
    ros::ServiceClient client 
        = n.serviceClient<grasp_srv::GraspGen>("grasp_gen");
    // Call Srv
    if(client.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
        // Publish
        grasps = srv.response.grasps;  // Update Response
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
    }
    return;
}


void ModelNameCallBack(const std_msgs::String::ConstPtr& msg) {
    model_name = msg->data;
    ROS_INFO("Model Name Updated.");
};


void GraspIdCallBack(const std_msgs::Int32::ConstPtr& msg, 
                     ros::NodeHandle n) {
    grasp_id = msg->data;
    ROS_INFO("Grasp ID Updated.");

    // Publish GraspPose
    bool flag = false;
    for(auto i = 0; i < grasps.global_grasp_poses.size(); ++i) {
        if(grasps.global_grasp_poses[i].model_names.size() == 0) continue;  // No Grasps Here
        std::string name = grasps.global_grasp_poses[i].model_names[0];
        if(name == model_name) {
            if(grasp_id >= grasps.global_grasp_poses[i].model_names.size()) 
                grasp_id = grasp_id % grasps.global_grasp_poses[i].model_names.size();
            geometry_msgs::Pose grasp_pose = 
                grasps.global_grasp_poses[i].grasp_poses[grasp_id];
            flag = true;
            // Publish Pose
            ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("grasp_pose", 1);
            pose_pub.publish(grasp_pose);
            break;
        }
    }
    if(!flag) ROS_INFO("No Poses For This Model");
};


void SaveCallBack(const std_msgs::Bool::ConstPtr& msg, const std::string& data_path) {
    // Save Local LocalPose and Scale
    std::string model_path = data_path + model_name;
};

int main(int argc, char **argv) 
{
    // Find Data path
    std::string scene_dir(argv[1]);  // Parse for Scene
    std::string model_dir(argv[2]);

    // ros init
    ros::init(argc, argv, "grasp_gen_client");
    ros::NodeHandle n;
    // Create Boost Function
    boost::function<void (const std_msgs::String::ConstPtr&)> f1 =
        boost::bind(SceneNameCallBack, boost::placeholders::_1, n, scene_dir);
    // Create Subscriber
    ros::Subscriber scene_sub  = n.subscribe<std_msgs::String>("scene_name", 1000, f1);
    
    // Grasp ID
    boost::function<void (const std_msgs::Int32::ConstPtr&)> f2 =
        boost::bind(GraspIdCallBack, boost::placeholders::_1, n);
    ros::Subscriber grasp_sub  = n.subscribe<std_msgs::Int32>("grasp_id", 1000, f2);
    
    // Model Name
    ros::Subscriber model_name_sub  = n.subscribe<std_msgs::String>("model_name", 1000, ModelNameCallBack);
    
    // Save 
    boost::function<void (const std_msgs::Bool::ConstPtr&)> f3 =
        boost::bind(SaveCallBack, boost::placeholders::_1, model_dir);
    ros::Subscriber save_sub  = n.subscribe<std_msgs::Bool>("save", 1000, f3);

    ros::MultiThreadedSpinner spinner(6); 
    spinner.spin();

    return 0;
}
