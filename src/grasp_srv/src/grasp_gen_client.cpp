#include "grasp_gen_client.hpp"


SceneManagement::SceneManagement(std::string scene_dir, std::string model_dir) :
    scene_dir_(scene_dir), model_dir_(model_dir), grasp_id_(0) {
    // Publish initialization
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("grasp_pose", 1);
    // Subscriber
    // Scene
    scene_sub_  = nh_.subscribe<std_msgs::String>("scene_name", 1000,
        &SceneNameCallBack, this); 
    // Grasp ID
    grasp_sub_  = nh_.subscribe<std_msgs::Int32>("grasp_id", 1000,
        &GraspIdCallBack, this);
    // Model Name
    model_name_sub_  = nh_.subscribe<std_msgs::String>("model_name", 1000,
        &ModelNameCallBack, this);
    // Save 
    save_sub_  = nh_.subscribe<std_msgs::Bool>("save_signal", 1000, 
        &SaveCallBack, this);
    
    // Start Service
    client_ = nh_.serviceClient<grasp_srv::GraspGen>("grasp_gen");
}

void SceneManagement::publish_pose() {
    bool flag = false;
    for(auto i = 0; i < grasps_.global_grasp_poses.size(); ++i) {
        if(grasps_.global_grasp_poses[i].model_names.size() == 0) continue;
        std::string name = grasps_.global_grasp_poses[i].model_names[0];
        if(name == model_name_) {
            if(grasp_id_ >= grasps_.global_grasp_poses[i].model_names.size()) 
                grasp_id_ = grasp_id_ % grasps_.global_grasp_poses[i].model_names.size();
            geometry_msgs::Pose grasp_pose = 
                grasps_.global_grasp_poses[i].grasp_poses[grasp_id_];
            flag = true;
            pose_pub_.publish(grasp_pose);
            break;
        }
    }
    if(!flag) ROS_INFO("No Poses For This Model.");
    else ROS_INFO("Pose Published.");
}

/*
When Receive a scene name, ask for grasp pose in the scene
*/
void SceneManagement::SceneNameCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Choose Scene: [%s]", msg->data.c_str());
    std::string scene_name_ = msg->data;
    std::string json_path = scene_dir_ + scene_name_ + "/object_data.json";
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
        std::string pd_filename = scene_dir_ + scene_name_ + "/" + std::to_string(id) + ".pcd";
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pd_filename, *cloud);
        // Transform this into a msg
        sensor_msgs::PointCloud2 point_cloud;
        pcl::toROSMsg<pcl::PointXYZRGBA>(*cloud, point_cloud);	
        srv.request.object_poses.object_point_clouds.push_back(point_cloud);
    }
    // Call Srv
    if(client_.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
        // Publish
        grasps_ = srv.response.grasps;  // Update Response
        // Update ModelName & Id
        model_name_ = grasps_.global_grasp_poses[0].model_names[0];
        grasp_id_ = 0;
        publish_pose();  // Publish Pose After Call
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
    }
    return;
}

void SceneManagement::ModelNameCallBack(const std_msgs::String::ConstPtr& msg) {
    model_name_ = msg->data;
    grasp_id_ = 0;  // Reset grasp_id_ after change model
    publish_pose();
    ROS_INFO("Model Name Updated.");
};

void SceneManagement::GraspIdCallBack(const std_msgs::Int32::ConstPtr& msg) {
    grasp_id_ = msg->data;
    publish_pose();   
    ROS_INFO("Grasp ID Updated.");
};

void SceneManagement::SaveCallBack(const std_msgs::Bool::ConstPtr& msg) {
    // Save Local LocalPose and Scale
    std::string model_path = model_dir_ + model_name_;
    std::string pose_file_name = model_path + "/pose.json";
    std::ifstream json_file(pose_file_name);
    json pose_datas;
    json_file >> pose_datas;
    json_file.close();
    // Add new pose
    for(auto i = 0; i < grasps_.global_grasp_poses.size(); ++i) {
        if(grasps_.global_grasp_poses[i].model_names.size() == 0) continue;  // No grasps_ Here
        std::string name = grasps_.global_grasp_poses[i].model_names[0];
        if(name == model_name_) {
            assert(grasp_id_ < grasps_.global_grasp_poses[i].model_names.size()); 
            geometry_msgs::Pose local_pose = 
                grasps_.global_grasp_poses[i].local_poses[grasp_id_];
            double scale = grasps_.global_grasp_poses[i].scales[grasp_id_];
            double grasp_width = grasps_.global_grasp_poses[i].grasp_widths[grasp_id_];
            std::vector<double> pose_array = {
                local_pose.position.x,
                local_pose.position.y,
                local_pose.position.z,
                local_pose.orientation.x,
                local_pose.orientation.y,
                local_pose.orientation.z,
                local_pose.orientation.w,
                scale,
                grasp_width};
            pose_datas.push_back(pose_array);
            std::ofstream out_file(pose_file_name);
            out_file << pose_datas;
            out_file.close();
            ROS_INFO("Grasp Saved.");
            return;
        }
    }
};

// Use Class to Manager it
int main(int argc, char **argv) 
{
    // ros init
    ros::init(argc, argv, "grasp_gen_client");
    // Find Data path
    std::string scene_dir(argv[1]);  // Parse for Scene
    std::string model_dir(argv[2]);
    SceneManagement scene_management(scene_dir, model_dir);

    ros::MultiThreadedSpinner spinner(4); 
    spinner.spin();
    return 0;
}
