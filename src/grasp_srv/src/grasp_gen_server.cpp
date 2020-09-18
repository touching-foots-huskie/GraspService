// CPP
#include <sstream>
#include <string>
#include <vector>

// ROS
#include "ros/ros.h"
#include "grasp_srv/GraspGen.h"

// grasp
#include <gpd/grasp_detector.h>


int main(int argc, char **argv) {
    // Get Grasp Generator
    std::string workspace_path(argv[1]);
    std::string model_dir(argv[2]);

    std::string config_filename = workspace_path + "/cfg/eigen_params.cfg";
    gpd::GraspDetector detector(config_filename, workspace_path, model_dir);
    
    ros::init(argc, argv, "grasp_gen_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("grasp_gen", 
                                                    &gpd::GraspDetector::grasp_gen,
                                                    &detector);
    ROS_INFO("Ready to generate grasp.");
    ros::spin();
    
    return 0;
}
