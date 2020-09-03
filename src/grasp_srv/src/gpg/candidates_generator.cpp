#include <cstdio>
#include <gpg/candidates_generator.h>


CandidatesGenerator::CandidatesGenerator(const Parameters& params,
  const HandSearch::Parameters& hand_search_params) : params_(params)
{
  Eigen::initParallel();

  hand_search_ = new HandSearch(hand_search_params);
}


void CandidatesGenerator::preprocessPointCloud(CloudCamera& cloud_cam)
{
//  const double VOXEL_SIZE = 0.002;
  const double VOXEL_SIZE = 0.003;

  std::cout << "Processing cloud with: " << cloud_cam.getCloudOriginal()->size() << " points.\n";

  // Calculate surface normals using integral images if possible.
  if (cloud_cam.getCloudOriginal()->isOrganized() && cloud_cam.getNormals().cols() == 0)
  {
    cloud_cam.calculateNormals(0);
  }

  // perform statistical outlier removal
  if (params_.remove_statistical_outliers_)
  {
    //    Plot plotter;
    //    plotter.drawCloud(cloud_cam.getCloudProcessed(), "before");

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_cam.getCloudProcessed());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_cam.getCloudProcessed());
    std::cout << "Cloud after removing statistical outliers: " << cloud_cam.getCloudProcessed()->size() << std::endl;
    //    plotter.drawCloud(cloud_cam.getCloudProcessed(), "after");
  }

  // No indices into point cloud given
  if (cloud_cam.getSampleIndices().size() == 0)
  {
    // 1. Workspace filtering
    cloud_cam.filterWorkspace(params_.workspace_);
    std::cout << "After workspace filtering: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";

    if (cloud_cam.getCloudProcessed()->size() == 0)
    {
      return;
    }

    // 2. Voxelization
    if (params_.voxelize_)
    {
      cloud_cam.voxelizeCloud(VOXEL_SIZE); // The problem lays here
      std::cout << "After voxelization: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";
    }

    // 3. Subsampling
    if (cloud_cam.getSamples().cols() > 0)
    {
      // 4. Calculate surface normals.
      if (cloud_cam.getNormals().cols() == 0)
      {
        cloud_cam.calculateNormals(params_.num_threads_);
      }

      // 5. Subsample the remaining samples.
      cloud_cam.subsampleSamples(params_.num_samples_);
    }
    else
    {
      if (params_.num_samples_ > cloud_cam.getCloudProcessed()->size())
      {
        std::vector<int> indices_all(cloud_cam.getCloudProcessed()->size());
        for (int i=0; i < cloud_cam.getCloudProcessed()->size(); i++)
          indices_all[i] = i;
        cloud_cam.setSampleIndices(indices_all);
        std::cout << "Cloud is smaller than num_samples. Subsampled all " << cloud_cam.getCloudProcessed()->size()
          << " points.\n";
      }
      else
      {
        cloud_cam.subsampleUniformly(params_.num_samples_);
        std::cout << "Subsampled " << params_.num_samples_ << " at random uniformly.\n";
      }
    }
  }
  // Indices into point cloud given
  else
  {
    if (params_.num_samples_ > 0 && params_.num_samples_ < cloud_cam.getSampleIndices().size())
    {
      std::vector<int> indices_rand(params_.num_samples_);
      for (int i=0; i < params_.num_samples_; i++)
        indices_rand[i] = cloud_cam.getSampleIndices()[rand() % cloud_cam.getSampleIndices().size()];
      cloud_cam.setSampleIndices(indices_rand);
      std::cout << "Subsampled " << indices_rand.size() << " indices.\n";
    }
    else
    {
      std::cout << "Using all " << cloud_cam.getSampleIndices().size() << " indices.\n";
    }
  }

  // 4. Calculate surface normals.
  if (cloud_cam.getNormals().cols() == 0)
  {
    cloud_cam.calculateNormals(params_.num_threads_);
  }

  if (params_.plot_normals_)
  {
    printf("Start printing Normals\n");
	  plotter_.plotNormals(cloud_cam.getCloudProcessed(), cloud_cam.getNormals());
  }
}


std::vector<Grasp> CandidatesGenerator::generateGraspCandidates(const CloudCamera& cloud_cam)
{
  // Find sets of grasp candidates. || Search
  std::vector<GraspSet> hand_set_list = hand_search_->searchHands(cloud_cam);
  std::cout << "Generated " << hand_set_list.size() << " grasp candidate sets.\n";

  // Extract the grasp candidates. || Grasp Representation
  std::vector<Grasp> candidates;
  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();

    for (int j = 0; j < hands.size(); j++)
    {
      if (hand_set_list[i].getIsValid()(j))
      {
        candidates.push_back(hands[j]);
      }
    }
  }
  std::cout << "Generated " << candidates.size() << " grasp candidates.\n";

  if (params_.plot_grasps_)
  {
    printf("Start Printing Normals\n");
    const HandSearch::Parameters& params = hand_search_->getParams();
    plotter_.plotFingers3D(candidates, cloud_cam.getCloudOriginal(), "Grasp Candidates", params.hand_outer_diameter_,
      params.finger_width_, params.hand_depth_, params.hand_height_);
  }

  return candidates;
}


std::vector<GraspSet> CandidatesGenerator::generateGraspCandidateSets(const CloudCamera& cloud_cam)
{
  // Find sets of grasp candidates.
  std::vector<GraspSet> hand_set_list = hand_search_->searchHands(cloud_cam);

  if (params_.plot_grasps_)
  {
    printf("Start printing fingers\n");
    const HandSearch::Parameters& params = hand_search_->getParams();
    plotter_.plotFingers3D(hand_set_list, cloud_cam.getCloudOriginal(), "Grasp Candidates", params.hand_outer_diameter_,
      params.finger_width_, params.hand_depth_, params.hand_height_);
  }

  return hand_set_list;
}


std::vector<Grasp> CandidatesGenerator::reevaluateHypotheses(const CloudCamera& cloud_cam,
  const std::vector<Grasp>& grasps)
{
  return hand_search_->reevaluateHypotheses(cloud_cam, grasps);
}


/*
Call Back Function for a ROS service
*/
bool CandidatesGenerator::grasp_gen(grasp_srv::GraspGen::Request  &req,
                                    grasp_srv::GraspGen::Response &res) {
  // std::string model_name = req.model_name;
  int object_num = req.object_poses.object_names.size();
  for(int obj_i = 0; obj_i < object_num; ++obj_i) {
    std::ostringstream model_path;
    std::string model_name = req.object_poses.object_names[obj_i];
    model_path << "/root/ocrtoc_materials/models/"
                 << model_name << "/"
                 << "meshes/";

    // If read true, then read instead of computing
    if(params_.msg_read_) {
      rosbag::Bag bag;
      std::string bag_path = model_path.str() + "grasp.bag";
      bag.open(bag_path, rosbag::bagmode::Read);
      for(rosbag::MessageInstance const m: rosbag::View(bag)) {
        grasp_srv::GlobalGraspPose::ConstPtr global_grasp_ptr 
          = m.instantiate<grasp_srv::GlobalGraspPose>();
        if (global_grasp_ptr != nullptr)
          res.grasps.global_grasp_poses.push_back(*global_grasp_ptr);
      }
      bag.close();
      continue; //   Go to the next object
    }

    std::string pcd_file_name = model_path.str() + "textured.pcd";
    // load pointcloud
    CloudCamera cloud_cam(pcd_file_name, view_points_);
    if (cloud_cam.getCloudOriginal()->size() == 0)
    {
      std::cout << "Input point cloud is empty or does not exist!\n";
      return (-1);
    }
    // load normals : if possible
    preprocessPointCloud(cloud_cam);

    ROS_INFO("Preprocess Finished");
    // generate grasp candidates
    std::vector<Grasp> candidates = generateGraspCandidates(cloud_cam);
    // Warp it into the msg
    if(candidates.size() < 1) {
      ROS_INFO("No Grasp Pose Found.");
    }
    else {
      ROS_INFO("Grasp Pose Found");      
      // Save it into the msg
      grasp_srv::GlobalGraspPose global_grasp_msg;  // msg for an object
      for(int grasp_i = 0; grasp_i < candidates.size(); ++grasp_i) {
        Grasp output_grasp = candidates[grasp_i];

        float score = output_grasp.getScore();
        bool full_antipodal = output_grasp.isFullAntipodal();
        bool half_antipodal = output_grasp.isHalfAntipodal();
        global_grasp_msg.scores.push_back(score);

        // Set Grasp Pose
        Eigen::Vector3d surface = output_grasp.getGraspSurface();
        Eigen::Vector3d bottom  = output_grasp.getGraspBottom();
        Eigen::Vector3d top     = output_grasp.getGraspTop();
        Eigen::Matrix3d frame   = output_grasp.getFrame();

        double pre_distance = 0.1;
        Eigen::Vector3d pre_bottom = bottom;
        Eigen::Vector3d pre_vector(0.0, -pre_distance, 0.0);
        pre_bottom += (frame * pre_vector);

        // Frame need to be change to accomdate the urdf
        Eigen::Matrix3d gpg2urdf;
        gpg2urdf = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX())
                 * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ());
        frame = frame * gpg2urdf;

        // Compute global pose
        geometry_msgs::Pose object_pose = req.object_poses.object_poses[obj_i];
        Eigen::Quaternion<double> object_frame_quat(object_pose.orientation.w,
                                                    object_pose.orientation.x,
                                                    object_pose.orientation.y,
                                                    object_pose.orientation.z);
        
        Eigen::Matrix3d object_frame_matrix = object_frame_quat.matrix();
        Eigen::Matrix3d grasp_frame = object_frame_matrix * frame;
        Eigen::Vector3d object_position(object_pose.position.x,
                                        object_pose.position.y,
                                        object_pose.position.z);

        Eigen::Vector3d grasp_point = object_frame_matrix * bottom + object_position;
        Eigen::Vector3d pre_grasp_point = object_frame_matrix * pre_bottom + object_position;

        Eigen::Quaternion<double> grasp_frame_quat(grasp_frame);

        geometry_msgs::Pose grasp_pose;
        geometry_msgs::Pose pre_grasp_pose;
        // grasp_pose
        grasp_pose.position.x = grasp_point.x();
        grasp_pose.position.y = grasp_point.y();
        grasp_pose.position.z = grasp_point.z();
        grasp_pose.orientation.x = grasp_frame_quat.x();
        grasp_pose.orientation.y = grasp_frame_quat.y();
        grasp_pose.orientation.z = grasp_frame_quat.z();
        grasp_pose.orientation.w = grasp_frame_quat.w();
        // pre_grasp_pose
        pre_grasp_pose.position.x = pre_grasp_point.x();
        pre_grasp_pose.position.y = pre_grasp_point.y();
        pre_grasp_pose.position.z = pre_grasp_point.z();
        pre_grasp_pose.orientation.x = grasp_frame_quat.x();
        pre_grasp_pose.orientation.y = grasp_frame_quat.y();
        pre_grasp_pose.orientation.z = grasp_frame_quat.z();
        pre_grasp_pose.orientation.w = grasp_frame_quat.w();

        global_grasp_msg.grasp_poses.push_back(grasp_pose);
        global_grasp_msg.pre_grasp_poses.push_back(pre_grasp_pose);


        // Set grasp width
        float grasp_width = output_grasp.getGraspWidth();
        global_grasp_msg.grasp_widths.push_back(grasp_width);
        global_grasp_msg.model_names.push_back(model_name);
      }
      // Add one more global grasp poses
      res.grasps.global_grasp_poses.push_back(global_grasp_msg);

      // Write MSG into corresponding dir
      if(params_.msg_write_) {
        rosbag::Bag bag;
        std::string bag_path = model_path.str() + "grasp.bag";
        bag.open(bag_path, rosbag::bagmode::Write);
        bag.write("grasp_per_object", ros::Time::now(), global_grasp_msg);
        bag.close();
      }
    } 
  }
  return true;  // Exit successfully
}
