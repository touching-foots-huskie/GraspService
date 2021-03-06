#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

//global frame id
std::string object_name = "a_cups";
Eigen::Affine3f object_transform = Eigen::Affine3f::Identity(); // float
float object_scale = 1.0;


// Object Name Call Back
void ObjectNameCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Choose Object: [%s]", msg->data.c_str());
    object_name = msg->data;
}


// Object Scale Call Back
void ObjectScaleCallBack(const std_msgs::Float64::ConstPtr& msg) {
    ROS_INFO("Scale: [%f]", msg->data);
    object_scale = msg->data;
}


void ObjectPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg) {
    Eigen::Quaternion<float> q(float(msg->orientation.w),
                               float(msg->orientation.x),
                               float(msg->orientation.y),
                               float(msg->orientation.z));
    Eigen::Vector3f t(float(msg->position.x),
                      float(msg->position.y),
                      float(msg->position.z));
    
    Eigen::Affine3f s;
    s = Eigen::Scaling(object_scale);
    // Update object transform
    object_transform = Eigen::Affine3f::Identity();
    object_transform.rotate(q).pretranslate(t);
    object_transform = object_transform * s;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr 
    pcd_read_file(std::string data_path,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud) {
    pcl::io::loadPCDFile<pcl::PointXYZ>(data_path, *point_cloud);
    // Pointcloud transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud<pcl::PointXYZ, float>
     (*point_cloud, *transformed_cloud, object_transform);
    return transformed_cloud;
}


int main(int argc, char** argv)
{
    std::string workspace_path;
    workspace_path = std::string(argv[1]);
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    // Model Subscriber
    ros::Subscriber object_sub = nh.subscribe("object_name", 1000, ObjectNameCallBack);
    ros::Subscriber pose_sub   = nh.subscribe("object_pose", 1000, ObjectPoseCallBack);
    ros::Subscriber scale_sub  = nh.subscribe("object_scale", 1000, ObjectScaleCallBack);
    // Point Cloud Publisher
    ros::Publisher object_pointcloud_pub 
        = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("object", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_msg(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        std::string object_path = workspace_path  
                                + object_name
                                + "/visual_meshes/cloud.pcd";
        // DepthCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pd_msg = pcd_read_file(object_path, object_msg);
        transformed_pd_msg->header.frame_id = "world";  // Deal with base frame
        pcl_conversions::toPCL(ros::Time::now(), transformed_pd_msg->header.stamp);
        object_pointcloud_pub.publish(transformed_pd_msg);

        ros::spinOnce ();
        loop_rate.sleep ();
    }
}