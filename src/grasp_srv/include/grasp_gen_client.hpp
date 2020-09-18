#ifndef GRASP_GEN_CLIENT
#define GRASP_GEN_CLIENT

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
#include "grasp_srv/SaveGrasp.h"

// json related 
#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

// stream
#include <istream>
#include <ostream>
#include <fstream>

// Eigen related
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// Boost
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>

// JSON
#include "json_tools.h"


class SceneManagement {
public:
    SceneManagement(std::string scene_dir, std::string grasp_dir);
    void SceneNameCallBack(const std_msgs::String::ConstPtr& msg);
    void ModelNameCallBack(const std_msgs::String::ConstPtr& msg);
    void GraspIdCallBack(const std_msgs::Int32::ConstPtr& msg);
    void SaveCallBack(const std_msgs::Bool::ConstPtr& msg);
    void GraspSaveCallBack(const grasp_srv::SaveGrasp::ConstPtr& msg);

private:
    void publish_pose();
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    ros::ServiceClient client_;
    ros::Subscriber scene_sub_;
    ros::Subscriber grasp_sub_;
    ros::Subscriber model_name_sub_;
    ros::Subscriber save_sub_;
    ros::Subscriber grasp_save_sub_;

    grasp_srv::Grasps grasps_;
    std::string model_name_;
    std::string scene_name_;
    int grasp_id_;
    std::string scene_dir_;
    std::string grasp_dir_;
};

#endif