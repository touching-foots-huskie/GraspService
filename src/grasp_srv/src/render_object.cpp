#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

//global frame id
std::string object_name = "a_cups";
Eigen::Affine3f object_transform = Eigen::Affine3f::Identity(); // float


// Object Name Call Back
void ObjectNameCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Choose Object: [%s]", msg->data.c_str());
    object_name = msg->data;
}


void ObjectPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg) {
    Eigen::Quaternion<float> q(float(msg->orientation.w),
                               float(msg->orientation.x),
                               float(msg->orientation.y),
                               float(msg->orientation.z));
    Eigen::Vector3f t(float(msg->position.x),
                      float(msg->position.y),
                      float(msg->position.z));
    // Update object transform
    object_transform = Eigen::Affine3f::Identity();
    object_transform.rotate(q).pretranslate(t);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr 
    pcd_read_file(std::string data_path,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud) {
    std::cout << "Data Path : " << data_path << std::endl;
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
    if(argc >= 2) {
      workspace_path = std::string(argv[1]);
    } else {
      workspace_path = "/root/GraspService/src/grasp_srv";
    }

    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    // Model Name Subscriber
    ros::Subscriber object_sub = nh.subscribe("object_name", 1000, ObjectNameCallBack);
    ros::Subscriber pose_sub   = nh.subscribe("object_pose", 1000, ObjectPoseCallBack);

    // Point Cloud Publisher
    ros::Publisher object_pointcloud_pub 
        = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("object", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_msg(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        std::string object_path = workspace_path 
                                + "/data/" 
                                + object_name
                                + "/meshes/textured.pcd";
        // DepthCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pd_msg = pcd_read_file(object_path, object_msg);
        transformed_pd_msg->header.frame_id = "world";  // Deal with base frame
        pcl_conversions::toPCL(ros::Time::now(), transformed_pd_msg->header.stamp);
        object_pointcloud_pub.publish(transformed_pd_msg);

        ros::spinOnce ();
        loop_rate.sleep ();
    }
}