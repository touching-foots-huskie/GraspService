#include <gpd/grasp_detector.h>

namespace gpd {

GraspDetector::GraspDetector(const std::string &config_filename) {
    Eigen::initParallel();

    // Read parameters from configuration file.
    util::ConfigFile config_file(config_filename);
    config_file.ExtractKeys();

    // Read hand geometry parameters.
    std::string hand_geometry_filename =
        config_file.getValueOfKeyAsString("hand_geometry_filename", "");
    if (hand_geometry_filename == "0") {
        hand_geometry_filename = config_filename;
    } 
    candidate::HandGeometry hand_geom(hand_geometry_filename);
    std::cout << hand_geom;

    // Read plotting parameters.
    plot_normals_ = config_file.getValueOfKey<bool>("plot_normals", false);
    plot_samples_ = config_file.getValueOfKey<bool>("plot_samples", true);
    plot_candidates_ = config_file.getValueOfKey<bool>("plot_candidates", false);
    plot_filtered_candidates_ =
        config_file.getValueOfKey<bool>("plot_filtered_candidates", false);
    plot_valid_grasps_ =
        config_file.getValueOfKey<bool>("plot_valid_grasps", false);
    plot_clustered_grasps_ =
        config_file.getValueOfKey<bool>("plot_clustered_grasps", false);
    plot_selected_grasps_ =
        config_file.getValueOfKey<bool>("plot_selected_grasps", false);
    printf("============ PLOTTING ========================\n");
    printf("plot_normals: %s\n", plot_normals_ ? "true" : "false");
    printf("plot_samples %s\n", plot_samples_ ? "true" : "false");
    printf("plot_candidates: %s\n", plot_candidates_ ? "true" : "false");
    printf("plot_filtered_candidates: %s\n",
            plot_filtered_candidates_ ? "true" : "false");
    printf("plot_valid_grasps: %s\n", plot_valid_grasps_ ? "true" : "false");
    printf("plot_clustered_grasps: %s\n",
            plot_clustered_grasps_ ? "true" : "false");
    printf("plot_selected_grasps: %s\n",
            plot_selected_grasps_ ? "true" : "false");
    printf("==============================================\n");

    // Create object to generate grasp candidates.
    candidate::CandidatesGenerator::Parameters generator_params;
    generator_params.num_samples_ =
        config_file.getValueOfKey<int>("num_samples", 1000);
    generator_params.num_threads_ =
        config_file.getValueOfKey<int>("num_threads", 1);
    generator_params.remove_statistical_outliers_ =
        config_file.getValueOfKey<bool>("remove_outliers", false);
    generator_params.sample_above_plane_ =
        config_file.getValueOfKey<bool>("sample_above_plane", false);
    generator_params.voxelize_ =
        config_file.getValueOfKey<bool>("voxelize", true);
    generator_params.voxel_size_ =
        config_file.getValueOfKey<double>("voxel_size", 0.003);
    generator_params.normals_radius_ =
        config_file.getValueOfKey<double>("normals_radius", 0.03);
    generator_params.refine_normals_k_ =
        config_file.getValueOfKey<int>("refine_normals_k", 0);
    generator_params.workspace_ =
        config_file.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");

    candidate::HandSearch::Parameters hand_search_params;
    hand_search_params.hand_geometry_ = hand_geom;
    hand_search_params.nn_radius_frames_ =
        config_file.getValueOfKey<double>("nn_radius", 0.01);
    hand_search_params.num_samples_ =
        config_file.getValueOfKey<int>("num_samples", 1000);
    hand_search_params.num_threads_ =
        config_file.getValueOfKey<int>("num_threads", 1);
    hand_search_params.num_orientations_ =
        config_file.getValueOfKey<int>("num_orientations", 8);
    hand_search_params.num_finger_placements_ =
        config_file.getValueOfKey<int>("num_finger_placements", 10);
    hand_search_params.deepen_hand_ =
        config_file.getValueOfKey<bool>("deepen_hand", true);
    hand_search_params.hand_axes_ =
        config_file.getValueOfKeyAsStdVectorInt("hand_axes", "2");
    hand_search_params.friction_coeff_ =
        config_file.getValueOfKey<double>("friction_coeff", 20.0);
    hand_search_params.min_viable_ =
        config_file.getValueOfKey<int>("min_viable", 6);
    candidates_generator_ = std::make_unique<candidate::CandidatesGenerator>(
        generator_params, hand_search_params);

    printf("============ CLOUD PREPROCESSING =============\n");
    printf("voxelize: %s\n", generator_params.voxelize_ ? "true" : "false");
    printf("voxel_size: %.3f\n", generator_params.voxel_size_);
    printf("remove_outliers: %s\n",
            generator_params.remove_statistical_outliers_ ? "true" : "false");
    printStdVector(generator_params.workspace_, "workspace");
    printf("sample_above_plane: %s\n",
            generator_params.sample_above_plane_ ? "true" : "false");
    printf("normals_radius: %.3f\n", generator_params.normals_radius_);
    printf("refine_normals_k: %d\n", generator_params.refine_normals_k_);
    printf("==============================================\n");

    printf("============ CANDIDATE GENERATION ============\n");
    printf("num_samples: %d\n", hand_search_params.num_samples_);
    printf("num_threads: %d\n", hand_search_params.num_threads_);
    printf("nn_radius: %3.2f\n", hand_search_params.nn_radius_frames_);
    printStdVector(hand_search_params.hand_axes_, "hand axes");
    printf("num_orientations: %d\n", hand_search_params.num_orientations_);
    printf("num_finger_placements: %d\n",
            hand_search_params.num_finger_placements_);
    printf("deepen_hand: %s\n",
            hand_search_params.deepen_hand_ ? "true" : "false");
    printf("friction_coeff: %3.2f\n", hand_search_params.friction_coeff_);
    printf("min_viable: %d\n", hand_search_params.min_viable_);
    printf("==============================================\n");

    // TODO: Set the camera position.
    //  Eigen::Matrix3Xd view_points(3,1);
    //  view_points << camera_position[0], camera_position[1], camera_position[2];

    // Read grasp image parameters.
    std::string image_geometry_filename =
        config_file.getValueOfKeyAsString("image_geometry_filename", "");
    if (image_geometry_filename == "0") {
        image_geometry_filename = config_filename;
    }
    descriptor::ImageGeometry image_geom(image_geometry_filename);
    std::cout << image_geom;

    // Read classification parameters and create classifier.
    std::string model_file = config_file.getValueOfKeyAsString("model_file", "");
    std::string weights_file =
        config_file.getValueOfKeyAsString("weights_file", "");
    if (!model_file.empty() || !weights_file.empty()) {
        int device = config_file.getValueOfKey<int>("device", 0);
        int batch_size = config_file.getValueOfKey<int>("batch_size", 1);
        classifier_ = net::Classifier::create(
            model_file, weights_file, static_cast<net::Classifier::Device>(device),
            batch_size);
        min_score_ = config_file.getValueOfKey<int>("min_score", 0);
        printf("============ CLASSIFIER ======================\n");
        printf("model_file: %s\n", model_file.c_str());
        printf("weights_file: %s\n", weights_file.c_str());
        printf("batch_size: %d\n", batch_size);
        printf("==============================================\n");
    }

    // Read additional grasp image creation parameters.
    bool remove_plane = config_file.getValueOfKey<bool>(
        "remove_plane_before_image_calculation", false);

    // Create object to create grasp images from grasp candidates (used for
    // classification).
    image_generator_ = std::make_unique<descriptor::ImageGenerator>(
        image_geom, hand_search_params.num_threads_,
        hand_search_params.num_orientations_, false, remove_plane);

    // Read grasp filtering parameters based on robot workspace and gripper width.
    workspace_grasps_ = config_file.getValueOfKeyAsStdVectorDouble(
        "workspace_grasps", "-1 1 -1 1 -1 1");
    min_aperture_ = config_file.getValueOfKey<double>("min_aperture", 0.0);
    max_aperture_ = config_file.getValueOfKey<double>("max_aperture", 0.085);
    printf("============ CANDIDATE FILTERING =============\n");
    printStdVector(workspace_grasps_, "candidate_workspace");
    printf("min_aperture: %3.4f\n", min_aperture_);
    printf("max_aperture: %3.4f\n", max_aperture_);
    printf("==============================================\n");

    // Read grasp filtering parameters based on approach direction.
    filter_approach_direction_ =
        config_file.getValueOfKey<bool>("filter_approach_direction", false);
    std::vector<double> approach =
        config_file.getValueOfKeyAsStdVectorDouble("direction", "1 0 0");
    direction_ << approach[0], approach[1], approach[2];
    thresh_rad_ = config_file.getValueOfKey<double>("thresh_rad", 2.3);

    // Read clustering parameters.
    int min_inliers = config_file.getValueOfKey<int>("min_inliers", 1);
    clustering_ = std::make_unique<Clustering>(min_inliers);
    cluster_grasps_ = min_inliers > 0 ? true : false;
    printf("============ CLUSTERING ======================\n");
    printf("min_inliers: %d\n", min_inliers);
    printf("==============================================\n\n");

    // Read grasp selection parameters.
    num_selected_ = config_file.getValueOfKey<int>("num_selected", 100);

    // Create plotter.
    plotter_ = std::make_unique<util::Plot>(hand_search_params.hand_axes_.size(),
                                            hand_search_params.num_orientations_);
}

// Used in OCRTOC
GraspDetector::GraspDetector(const std::string& config_filename, 
                             const std::string& ws_path, 
                             const std::string& model_dir,
                             const std::string& grasp_dir) : 
                             ws_path_(ws_path), model_dir_(model_dir), grasp_dir_(grasp_dir) 
                             {

    Eigen::initParallel();
    // Read parameters from configuration file.
    util::ConfigFile config_file(config_filename);
    config_file.ExtractKeys();

    // Read hand geometry parameters.
    std::string hand_geometry_filename =
        config_file.getValueOfKeyAsString("hand_geometry_filename", "");
    if (hand_geometry_filename == "") {
        hand_geometry_filename = config_filename;
    } 
    else {
        hand_geometry_filename = ws_path + hand_geometry_filename;
    }
    candidate::HandGeometry hand_geom(hand_geometry_filename);
    std::cout << hand_geom;

    // Read plotting parameters.
    plot_normals_ = config_file.getValueOfKey<bool>("plot_normals", false);
    plot_samples_ = config_file.getValueOfKey<bool>("plot_samples", true);
    plot_candidates_ = config_file.getValueOfKey<bool>("plot_candidates", false);
    plot_filtered_candidates_ =
        config_file.getValueOfKey<bool>("plot_filtered_candidates", false);
    plot_valid_grasps_ =
        config_file.getValueOfKey<bool>("plot_valid_grasps", false);
    plot_clustered_grasps_ =
        config_file.getValueOfKey<bool>("plot_clustered_grasps", false);
    plot_selected_grasps_ =
        config_file.getValueOfKey<bool>("plot_selected_grasps", false);
    printf("============ PLOTTING ========================\n");
    printf("plot_normals: %s\n", plot_normals_ ? "true" : "false");
    printf("plot_samples %s\n", plot_samples_ ? "true" : "false");
    printf("plot_candidates: %s\n", plot_candidates_ ? "true" : "false");
    printf("plot_filtered_candidates: %s\n",
            plot_filtered_candidates_ ? "true" : "false");
    printf("plot_valid_grasps: %s\n", plot_valid_grasps_ ? "true" : "false");
    printf("plot_clustered_grasps: %s\n",
            plot_clustered_grasps_ ? "true" : "false");
    printf("plot_selected_grasps: %s\n",
            plot_selected_grasps_ ? "true" : "false");
    printf("==============================================\n");

    // Create object to generate grasp candidates.
    candidate::CandidatesGenerator::Parameters generator_params;
    generator_params.num_samples_ =
        config_file.getValueOfKey<int>("num_samples", 1000);
    generator_params.num_threads_ =
        config_file.getValueOfKey<int>("num_threads", 1);
    generator_params.remove_statistical_outliers_ =
        config_file.getValueOfKey<bool>("remove_outliers", false);
    generator_params.sample_above_plane_ =
        config_file.getValueOfKey<bool>("sample_above_plane", false);
    generator_params.voxelize_ =
        config_file.getValueOfKey<bool>("voxelize", true);
    generator_params.voxel_size_ =
        config_file.getValueOfKey<double>("voxel_size", 0.003);
    generator_params.normals_radius_ =
        config_file.getValueOfKey<double>("normals_radius", 0.03);
    generator_params.refine_normals_k_ =
        config_file.getValueOfKey<int>("refine_normals_k", 0);
    generator_params.workspace_ =
        config_file.getValueOfKeyAsStdVectorDouble("workspace", "-1 1 -1 1 -1 1");

    candidate::HandSearch::Parameters hand_search_params;
    hand_search_params.hand_geometry_ = hand_geom;
    hand_search_params.nn_radius_frames_ =
        config_file.getValueOfKey<double>("nn_radius", 0.01);
    hand_search_params.num_samples_ =
        config_file.getValueOfKey<int>("num_samples", 1000);
    hand_search_params.num_threads_ =
        config_file.getValueOfKey<int>("num_threads", 1);
    hand_search_params.num_orientations_ =
        config_file.getValueOfKey<int>("num_orientations", 8);
    hand_search_params.num_finger_placements_ =
        config_file.getValueOfKey<int>("num_finger_placements", 10);
    hand_search_params.deepen_hand_ =
        config_file.getValueOfKey<bool>("deepen_hand", true);
    hand_search_params.hand_axes_ =
        config_file.getValueOfKeyAsStdVectorInt("hand_axes", "2");
    hand_search_params.friction_coeff_ =
        config_file.getValueOfKey<double>("friction_coeff", 20.0);
    hand_search_params.min_viable_ =
        config_file.getValueOfKey<int>("min_viable", 6);
    candidates_generator_ = std::make_unique<candidate::CandidatesGenerator>(
        generator_params, hand_search_params);

    printf("============ CLOUD PREPROCESSING =============\n");
    printf("voxelize: %s\n", generator_params.voxelize_ ? "true" : "false");
    printf("voxel_size: %.3f\n", generator_params.voxel_size_);
    printf("remove_outliers: %s\n",
            generator_params.remove_statistical_outliers_ ? "true" : "false");
    printStdVector(generator_params.workspace_, "workspace");
    printf("sample_above_plane: %s\n",
            generator_params.sample_above_plane_ ? "true" : "false");
    printf("normals_radius: %.3f\n", generator_params.normals_radius_);
    printf("refine_normals_k: %d\n", generator_params.refine_normals_k_);
    printf("==============================================\n");

    printf("============ CANDIDATE GENERATION ============\n");
    printf("num_samples: %d\n", hand_search_params.num_samples_);
    printf("num_threads: %d\n", hand_search_params.num_threads_);
    printf("nn_radius: %3.2f\n", hand_search_params.nn_radius_frames_);
    printStdVector(hand_search_params.hand_axes_, "hand axes");
    printf("num_orientations: %d\n", hand_search_params.num_orientations_);
    printf("num_finger_placements: %d\n",
            hand_search_params.num_finger_placements_);
    printf("deepen_hand: %s\n",
            hand_search_params.deepen_hand_ ? "true" : "false");
    printf("friction_coeff: %3.2f\n", hand_search_params.friction_coeff_);
    printf("min_viable: %d\n", hand_search_params.min_viable_);
    printf("==============================================\n");

    // TODO: Set the camera position.
    //  Eigen::Matrix3Xd view_points(3,1);
    //  view_points << camera_position[0], camera_position[1], camera_position[2];

    // Read grasp image parameters.
    std::string image_geometry_filename =
        config_file.getValueOfKeyAsString("image_geometry_filename", "");
    if (image_geometry_filename == "") {
        image_geometry_filename = config_filename;
    }
    else {
        image_geometry_filename = ws_path + image_geometry_filename;
    }
    descriptor::ImageGeometry image_geom(image_geometry_filename);
    std::cout << image_geom;

    // Read classification parameters and create classifier.
    std::string model_file = config_file.getValueOfKeyAsString("model_file", "");
    if(!model_file.empty())
        model_file = ws_path + model_file;
    std::string weights_file =
        config_file.getValueOfKeyAsString("weights_file", "");
    if(!weights_file.empty())
        weights_file = ws_path + weights_file;
    if (!model_file.empty() || !weights_file.empty()) {
        int device = config_file.getValueOfKey<int>("device", 0);
        int batch_size = config_file.getValueOfKey<int>("batch_size", 1);
        classifier_ = net::Classifier::create(
            model_file, weights_file, static_cast<net::Classifier::Device>(device),
            batch_size);
        min_score_ = config_file.getValueOfKey<int>("min_score", 0);
        printf("============ CLASSIFIER ======================\n");
        printf("model_file: %s\n", model_file.c_str());
        printf("weights_file: %s\n", weights_file.c_str());
        printf("batch_size: %d\n", batch_size);
        printf("==============================================\n");
    }

    // Read additional grasp image creation parameters.
    bool remove_plane = config_file.getValueOfKey<bool>(
        "remove_plane_before_image_calculation", false);

    // Create object to create grasp images from grasp candidates (used for
    // classification).
    image_generator_ = std::make_unique<descriptor::ImageGenerator>(
        image_geom, hand_search_params.num_threads_,
        hand_search_params.num_orientations_, false, remove_plane);

    // Read grasp filtering parameters based on robot workspace and gripper width.
    workspace_grasps_ = config_file.getValueOfKeyAsStdVectorDouble(
        "workspace_grasps", "-1 1 -1 1 -1 1");
    min_aperture_ = config_file.getValueOfKey<double>("min_aperture", 0.0);
    max_aperture_ = config_file.getValueOfKey<double>("max_aperture", 0.085);
    printf("============ CANDIDATE FILTERING =============\n");
    printStdVector(workspace_grasps_, "candidate_workspace");
    printf("min_aperture: %3.4f\n", min_aperture_);
    printf("max_aperture: %3.4f\n", max_aperture_);
    printf("==============================================\n");

    // Read grasp filtering parameters based on approach direction.
    filter_approach_direction_ =
        config_file.getValueOfKey<bool>("filter_approach_direction", false);
    std::vector<double> approach =
        config_file.getValueOfKeyAsStdVectorDouble("direction", "1 0 0");
    direction_ << approach[0], approach[1], approach[2];
    thresh_rad_ = config_file.getValueOfKey<double>("thresh_rad", 2.3);

    // Read clustering parameters.
    int min_inliers = config_file.getValueOfKey<int>("min_inliers", 1);
    clustering_ = std::make_unique<Clustering>(min_inliers);
    cluster_grasps_ = min_inliers > 0 ? true : false;
    printf("============ CLUSTERING ======================\n");
    printf("min_inliers: %d\n", min_inliers);
    printf("==============================================\n\n");

    // Read grasp selection parameters.
    num_selected_ = config_file.getValueOfKey<int>("num_selected", 100);

    // Create plotter.
    plotter_ = std::make_unique<util::Plot>(hand_search_params.hand_axes_.size(),
                                            hand_search_params.num_orientations_);
   // OCRTOC Related
    centered_at_origin_ = config_file.getValueOfKey<bool>("centered_at_origin", false);
    pre_defined_enable_ = config_file.getValueOfKey<bool>("pre_defined_enable", false);
    geometry_enable_ = config_file.getValueOfKey<bool>("geometry_enable", false);
    flip_enable_ = config_file.getValueOfKey<bool>("flip_enable", false);
    gpd_enable_ = config_file.getValueOfKey<bool>("gpd_enable", false);
}

std::vector<std::unique_ptr<candidate::Hand>> GraspDetector::detectGrasps(
    const util::Cloud &cloud) {
    double t0_total = omp_get_wtime();
    std::vector<std::unique_ptr<candidate::Hand>> hands_out;

    const candidate::HandGeometry &hand_geom =
        candidates_generator_->getHandSearchParams().hand_geometry_;

    // Check if the point cloud is empty.
    if (cloud.getCloudOriginal()->size() == 0) {
        printf("ERROR: Point cloud is empty!");
        hands_out.resize(0);
        return hands_out;
    }

    // Plot samples/indices.
    if (plot_samples_) {
        if (cloud.getSamples().cols() > 0) {
        plotter_->plotSamples(cloud.getSamples(), cloud.getCloudProcessed());
        } else if (cloud.getSampleIndices().size() > 0) {
        plotter_->plotSamples(cloud.getSampleIndices(),
                                cloud.getCloudProcessed());
        }
    }

    if (plot_normals_) {
        std::cout << "Plotting normals for different camera sources\n";
        plotter_->plotNormals(cloud);
    }

    // 1. Generate grasp candidates.
    double t0_candidates = omp_get_wtime();
    std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list =
        candidates_generator_->generateGraspCandidateSets(cloud);
    printf("Generated %zu hand sets.\n", hand_set_list.size());
    if (hand_set_list.size() == 0) {
        return hands_out;
    }
    double t_candidates = omp_get_wtime() - t0_candidates;
    if (plot_candidates_) {
        plotter_->plotFingers3D(hand_set_list, cloud.getCloudOriginal(),
                                "Grasp candidates", hand_geom);
    }

    // 2. Filter the candidates.
    double t0_filter = omp_get_wtime();
    std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_filtered =
        filterGraspsWorkspace(hand_set_list, workspace_grasps_);
    if (hand_set_list_filtered.size() == 0) {
        return hands_out;
    }
    if (plot_filtered_candidates_) {
        plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
                                "Filtered Grasps (Aperture, Workspace)", hand_geom);
    }
    if (filter_approach_direction_) {
        hand_set_list_filtered =
            filterGraspsDirection(hand_set_list_filtered, direction_, thresh_rad_);
        if (plot_filtered_candidates_) {
        plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
                                "Filtered Grasps (Approach)", hand_geom);
        }
    }
    double t_filter = omp_get_wtime() - t0_filter;
    if (hand_set_list_filtered.size() == 0) {
        return hands_out;
    }

    // 3. Create grasp descriptors (images).
    double t0_images = omp_get_wtime();
    std::vector<std::unique_ptr<candidate::Hand>> hands;
    std::vector<std::unique_ptr<cv::Mat>> images;
    image_generator_->createImages(cloud, hand_set_list_filtered, images, hands);
    double t_images = omp_get_wtime() - t0_images;

    // 4. Classify the grasp candidates.
    double t0_classify = omp_get_wtime();
    std::vector<float> scores = classifier_->classifyImages(images);
    for (int i = 0; i < hands.size(); i++) {
        hands[i]->setScore(scores[i]);
    }
    double t_classify = omp_get_wtime() - t0_classify;

    // 5. Select the <num_selected> highest scoring grasps.
    hands = selectGrasps(hands);
    if (plot_valid_grasps_) {
        plotter_->plotFingers3D(hands, cloud.getCloudOriginal(), "Valid Grasps",
                                hand_geom);
    }

    // 6. Cluster the grasps.
    double t0_cluster = omp_get_wtime();
    std::vector<std::unique_ptr<candidate::Hand>> clusters;
    if (cluster_grasps_) {
        clusters = clustering_->findClusters(hands);
        printf("Found %d clusters.\n", (int)clusters.size());
        if (clusters.size() <= 3) {
        printf(
            "Not enough clusters found! Adding all grasps from previous step.");
        for (int i = 0; i < hands.size(); i++) {
            clusters.push_back(std::move(hands[i]));
        }
        }
        if (plot_clustered_grasps_) {
        plotter_->plotFingers3D(clusters, cloud.getCloudOriginal(),
                                "Clustered Grasps", hand_geom);
        }
    } else {
        clusters = std::move(hands);
    }
    double t_cluster = omp_get_wtime() - t0_cluster;

    // 7. Sort grasps by their score.
    std::sort(clusters.begin(), clusters.end(), isScoreGreater);
    printf("======== Selected grasps ========\n");
    for (int i = 0; i < clusters.size(); i++) {
        std::cout << "Grasp " << i << ": " << clusters[i]->getScore() << "\n";
    }
    printf("Selected the %d best grasps.\n", (int)clusters.size());
    double t_total = omp_get_wtime() - t0_total;

    printf("======== RUNTIMES ========\n");
    printf(" 1. Candidate generation: %3.4fs\n", t_candidates);
    printf(" 2. Descriptor extraction: %3.4fs\n", t_images);
    printf(" 3. Classification: %3.4fs\n", t_classify);
    // printf(" Filtering: %3.4fs\n", t_filter);
    // printf(" Clustering: %3.4fs\n", t_cluster);
    printf("==========\n");
    printf(" TOTAL: %3.4fs\n", t_total);

    if (plot_selected_grasps_) {
        plotter_->plotFingers3D(clusters, cloud.getCloudOriginal(),
                                "Selected Grasps", hand_geom, false);
    }

    return clusters;
}

void GraspDetector::preprocessPointCloud(util::Cloud &cloud) {
    candidates_generator_->preprocessPointCloud(cloud);
}

std::vector<std::unique_ptr<candidate::HandSet>>
GraspDetector::filterGraspsWorkspace(
    std::vector<std::unique_ptr<candidate::HandSet>> &hand_set_list,
    const std::vector<double> &workspace) const {
    int remaining = 0;
    std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_out;
    printf("Filtering grasps outside of workspace ...\n");

    const candidate::HandGeometry &hand_geometry =
        candidates_generator_->getHandSearchParams().hand_geometry_;

    for (int i = 0; i < hand_set_list.size(); i++) {
        const std::vector<std::unique_ptr<candidate::Hand>> &hands =
            hand_set_list[i]->getHands();
        Eigen::Array<bool, 1, Eigen::Dynamic> is_valid =
            hand_set_list[i]->getIsValid();

        for (int j = 0; j < hands.size(); j++) {
        if (!is_valid(j)) {
            continue;
        }
        double half_width = 0.5 * hand_geometry.outer_diameter_;
        Eigen::Vector3d left_bottom =
            hands[j]->getPosition() + half_width * hands[j]->getBinormal();
        Eigen::Vector3d right_bottom =
            hands[j]->getPosition() - half_width * hands[j]->getBinormal();
        Eigen::Vector3d left_top =
            left_bottom + hand_geometry.depth_ * hands[j]->getApproach();
        Eigen::Vector3d right_top =
            left_bottom + hand_geometry.depth_ * hands[j]->getApproach();
        Eigen::Vector3d approach =
            hands[j]->getPosition() - 0.05 * hands[j]->getApproach();
        Eigen::VectorXd x(5), y(5), z(5);
        x << left_bottom(0), right_bottom(0), left_top(0), right_top(0),
            approach(0);
        y << left_bottom(1), right_bottom(1), left_top(1), right_top(1),
            approach(1);
        z << left_bottom(2), right_bottom(2), left_top(2), right_top(2),
            approach(2);

        // Ensure the object fits into the hand and avoid grasps outside the
        // workspace.
        if (hands[j]->getGraspWidth() >= min_aperture_ &&
            hands[j]->getGraspWidth() <= max_aperture_ &&
            x.minCoeff() >= workspace[0] && x.maxCoeff() <= workspace[1] &&
            y.minCoeff() >= workspace[2] && y.maxCoeff() <= workspace[3] &&
            z.minCoeff() >= workspace[4] && z.maxCoeff() <= workspace[5]) {
            is_valid(j) = true;
            remaining++;
        } else {
            is_valid(j) = false;
        }
        }

        if (is_valid.any()) {
        hand_set_list_out.push_back(std::move(hand_set_list[i]));
        hand_set_list_out[hand_set_list_out.size() - 1]->setIsValid(is_valid);
        }
    }

    printf("Number of grasp candidates within workspace and gripper width: %d\n",
            remaining);

    return hand_set_list_out;
}

std::vector<std::unique_ptr<candidate::HandSet>>
GraspDetector::generateGraspCandidates(const util::Cloud &cloud) {
    return candidates_generator_->generateGraspCandidateSets(cloud);
}

std::vector<std::unique_ptr<candidate::Hand>> GraspDetector::selectGrasps(
    std::vector<std::unique_ptr<candidate::Hand>> &hands) const {
    printf("Selecting the %d highest scoring grasps ...\n", num_selected_);

    int middle = std::min((int)hands.size(), num_selected_);
    std::partial_sort(hands.begin(), hands.begin() + middle, hands.end(),
                        isScoreGreater);
    std::vector<std::unique_ptr<candidate::Hand>> hands_out;

    for (int i = 0; i < middle; i++) {
        hands_out.push_back(std::move(hands[i]));
        printf(" grasp #%d, score: %3.4f\n", i, hands_out[i]->getScore());
    }

    return hands_out;
}

std::vector<std::unique_ptr<candidate::HandSet>>
GraspDetector::filterGraspsDirection(
    std::vector<std::unique_ptr<candidate::HandSet>> &hand_set_list,
    const Eigen::Vector3d &direction, const double thresh_rad) {
    std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_out;
    int remaining = 0;

    for (int i = 0; i < hand_set_list.size(); i++) {
        const std::vector<std::unique_ptr<candidate::Hand>> &hands =
            hand_set_list[i]->getHands();
        Eigen::Array<bool, 1, Eigen::Dynamic> is_valid =
            hand_set_list[i]->getIsValid();

        for (int j = 0; j < hands.size(); j++) {
        if (is_valid(j)) {
            double angle = acos(direction.transpose() * hands[j]->getApproach());
            if (angle > thresh_rad) {
            is_valid(j) = false;
            } else {
            remaining++;
            }
        }
        }

        if (is_valid.any()) {
        hand_set_list_out.push_back(std::move(hand_set_list[i]));
        hand_set_list_out[hand_set_list_out.size() - 1]->setIsValid(is_valid);
        }
    }

    printf("Number of grasp candidates with correct approach direction: %d\n",
            remaining);

    return hand_set_list_out;
}

bool GraspDetector::createGraspImages(
    util::Cloud &cloud,
    std::vector<std::unique_ptr<candidate::Hand>> &hands_out,
    std::vector<std::unique_ptr<cv::Mat>> &images_out) {
    // Check if the point cloud is empty.
    if (cloud.getCloudOriginal()->size() == 0) {
        printf("ERROR: Point cloud is empty!");
        hands_out.resize(0);
        images_out.resize(0);
        return false;
    }

    // Plot samples/indices.
    if (plot_samples_) {
        if (cloud.getSamples().cols() > 0) {
        plotter_->plotSamples(cloud.getSamples(), cloud.getCloudProcessed());
        } else if (cloud.getSampleIndices().size() > 0) {
        plotter_->plotSamples(cloud.getSampleIndices(),
                                cloud.getCloudProcessed());
        }
    }

    if (plot_normals_) {
        std::cout << "Plotting normals for different camera sources\n";
        plotter_->plotNormals(cloud);
    }

    // 1. Generate grasp candidates.
    std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list =
        candidates_generator_->generateGraspCandidateSets(cloud);
    printf("Generated %zu hand sets.\n", hand_set_list.size());
    if (hand_set_list.size() == 0) {
        hands_out.resize(0);
        images_out.resize(0);
        return false;
    }

    const candidate::HandGeometry &hand_geom =
        candidates_generator_->getHandSearchParams().hand_geometry_;

    // 2. Filter the candidates.
    std::vector<std::unique_ptr<candidate::HandSet>> hand_set_list_filtered =
        filterGraspsWorkspace(hand_set_list, workspace_grasps_);
    if (plot_filtered_candidates_) {
        plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
                                "Filtered Grasps (Aperture, Workspace)", hand_geom);
    }
    if (filter_approach_direction_) {
        hand_set_list_filtered =
            filterGraspsDirection(hand_set_list_filtered, direction_, thresh_rad_);
        if (plot_filtered_candidates_) {
        plotter_->plotFingers3D(hand_set_list_filtered, cloud.getCloudOriginal(),
                                "Filtered Grasps (Approach)", hand_geom);
        }
    }

    // 3. Create grasp descriptors (images).
    std::vector<std::unique_ptr<candidate::Hand>> hands;
    std::vector<std::unique_ptr<cv::Mat>> images;
    image_generator_->createImages(cloud, hand_set_list_filtered, images_out,
                                    hands_out);

    return true;
}

std::vector<int> GraspDetector::evalGroundTruth(
    const util::Cloud &cloud_gt,
    std::vector<std::unique_ptr<candidate::Hand>> &hands) {
    return candidates_generator_->reevaluateHypotheses(cloud_gt, hands);
}

std::vector<std::unique_ptr<candidate::Hand>>
GraspDetector::pruneGraspCandidates(
    const util::Cloud &cloud,
    const std::vector<std::unique_ptr<candidate::HandSet>> &hand_set_list,
    double min_score) {
    // 1. Create grasp descriptors (images).
    std::vector<std::unique_ptr<candidate::Hand>> hands;
    std::vector<std::unique_ptr<cv::Mat>> images;
    image_generator_->createImages(cloud, hand_set_list, images, hands);

    // 2. Classify the grasp candidates.
    std::vector<float> scores = classifier_->classifyImages(images);
    std::vector<std::unique_ptr<candidate::Hand>> hands_out;

    // 3. Only keep grasps with a score larger than <min_score>.
    for (int i = 0; i < hands.size(); i++) {
        if (scores[i] > min_score) {
        hands[i]->setScore(scores[i]);
        hands_out.push_back(std::move(hands[i]));
        }
    }

    return hands_out;
}

void GraspDetector::printStdVector(const std::vector<int> &v,
                                   const std::string &name) const {
    printf("%s: ", name.c_str());
    for (int i = 0; i < v.size(); i++) {
        printf("%d ", v[i]);
    }
    printf("\n");
}

void GraspDetector::printStdVector(const std::vector<double> &v,
                                   const std::string &name) const {
    printf("%s: ", name.c_str());
    for (int i = 0; i < v.size(); i++) {
        printf("%3.2f ", v[i]);
    }
    printf("\n");
}

// OCRTOC related
bool GraspDetector::grasp_gen(grasp_srv::GraspGen::Request  &req,
                              grasp_srv::GraspGen::Response &res) {
    int object_num = req.object_poses.object_names.size();
    // local parameters
    double compensate_distance = 0.1;
    double pre_distance = 0.15;

    for(int obj_i = 0; obj_i < object_num; ++obj_i) {
        std::string model_name = req.object_poses.object_names[obj_i];
        double model_scale = req.object_poses.object_scales[obj_i];
        // Get Object Pose  
        geometry_msgs::Pose object_pose = req.object_poses.object_poses[obj_i];
        Eigen::Quaternion<double> object_frame_quat(object_pose.orientation.w,
                                                    object_pose.orientation.x,
                                                    object_pose.orientation.y,
                                                    object_pose.orientation.z);
        Eigen::Matrix3d object_frame_matrix = object_frame_quat.matrix();
        Eigen::Vector3d object_position(object_pose.position.x,
                                        object_pose.position.y,
                                        object_pose.position.z);

        grasp_srv::GlobalGraspPose global_grasp_msg;
        
        // Enable geometry pre-defined pose
        if(geometry_enable_) {
            // type filename
            std::string pose_dir;
            std::string type_filename;
            pose_dir = grasp_dir_ + model_name;
            // where the grasp file saved
            type_filename = pose_dir + "/type.json";

            // model filename
            std::ostringstream model_path;
            model_path << model_dir_ 
                       << model_name << "/visual_meshes/";
            std::string pcd_file_name = model_path.str() + "cloud.pcd";

            try {
                // Check Directory Existense 
                if(!exists_file(pose_dir)) {
                    boost::filesystem::create_directories(pose_dir); 
                }

                // File Check
                if(!exists_file(type_filename)) {
                    std::ofstream ofs;
                    ofs.open(type_filename, std::ofstream::out);
                    ofs << "{\"type\": \"unknown\"}";
                    ofs.close(); 
                }

                std::ifstream json_file(type_filename);
                json type_data_json;
                json_file >> type_data_json;
                json_file.close();
                std::string type_name = type_data_json["type"];

                // generate grasp pose
                VectorArray position_array;
                MatrixArray frame_array;
                if(type_name == "box") {
                    box_grasp(frame_array, position_array, pcd_file_name, model_scale);
                }
                else if(type_name == "can") {
                    can_grasp(frame_array, position_array, pcd_file_name, model_scale);
                }
                std::cout << "Grasp SIZE: " << frame_array.size() << std::endl;
                // generate msg
                for(int gi = 0; gi < frame_array.size(); ++gi) {
                    Eigen::Matrix3d frame = frame_array[gi];
                    Eigen::Vector3d bottom = position_array[gi];
                    // Generate Msg
                    float grasp_width = 0.12;  // default width
                    generate_msg(global_grasp_msg,  
                                 model_name, model_scale, grasp_width,
                                 frame, bottom,
                                 object_frame_matrix,
                                 object_position,
                                 false);
                    if(flip_enable_) {
                        // generate another with symmetric over x-axis
                        Eigen::AngleAxis<double> flip(1.0*M_PI, Eigen::Vector3d(1.,0.,0.));
                        frame = frame * flip.toRotationMatrix();
                        generate_msg(global_grasp_msg, 
                                    model_name, model_scale, grasp_width,
                                    frame, bottom,
                                    object_frame_matrix,
                                    object_position,
                                    false);
                    }
                }
                std::cout << "Grasp SIZE: " << global_grasp_msg.grasp_poses.size() << std::endl;
            }
            catch(std::exception& e) {
                ROS_INFO("Fail Pre-defined Pose.");
            }
        }

        // Load Predefined Pose
        if(pre_defined_enable_) {
            std::string pose_dir;
            std::string pose_filename;
            pose_dir = grasp_dir_ + model_name;
            // where the grasp file saved
            pose_filename = pose_dir + "/pose.json";
            try {
                // Check Directory Existense 
                if(!exists_file(pose_dir)) {
                    boost::filesystem::create_directories(pose_dir); 
                }

                // File Check
                if(!exists_file(pose_filename)) {
                    std::ofstream ofs;
                    ofs.open(pose_filename, std::ofstream::out);
                    ofs << "[]";
                    ofs.close(); 
                }

                std::ifstream json_file(pose_filename);
                json pose_datas_json;
                json_file >> pose_datas_json;
                json_file.close();

                // Save pose_datas into msg
                for(auto& [key, pose_data] : pose_datas_json.items()) {
                    Eigen::Vector3d bottom(pose_data[0],
                                           pose_data[1],
                                           pose_data[2]);
                    Eigen::Quaternion<double> frame_quat(pose_data[6],
                                                         pose_data[3],
                                                         pose_data[4],
                                                         pose_data[5]);
                    Eigen::Matrix3d frame = frame_quat.matrix();
                    double pre_scale = pose_data[7];
                    double relative_scale = model_scale / pre_scale;
                    // Generate Msg
                    float grasp_width = pose_data[8];
                    generate_msg(global_grasp_msg, 
                                 model_name, model_scale, grasp_width,
                                 frame, bottom,
                                 object_frame_matrix,
                                 object_position,
                                 false,
                                 relative_scale);
                    if(flip_enable_) {
                        // generate another with symmetric over x-axis
                        Eigen::AngleAxis<double> flip(1.0*M_PI, Eigen::Vector3d(1.,0.,0.));
                        frame = frame * flip.toRotationMatrix();
                        generate_msg(global_grasp_msg, 
                                    model_name, model_scale, grasp_width,
                                    frame, bottom,
                                    object_frame_matrix,
                                    object_position,
                                    false,
                                    relative_scale);
                    }
                }
                ROS_INFO("Add Pre-defined Pose.");
            }
            catch(std::exception& e) {
                ROS_INFO("Fail Pre-defined Pose.");
            }
        }

        if(gpd_enable_) {
            // Get PointCloud First
            util::Cloud cloud;
            Eigen::Matrix<double, 3, 1> default_view_point;
            default_view_point << 0., 0., 0.;
            if(model_name == "raw_pointcloud") {
                pcl::PointCloud<pcl::PointXYZRGBA> input_pointcloud;
                pcl::fromROSMsg<pcl::PointXYZRGBA>(req.object_poses.object_point_clouds[obj_i],
                                                    input_pointcloud);	
                // TODO: support dynamic view points
                cloud.setPointCloud(input_pointcloud.makeShared(),
                                    default_view_point,
                                    model_scale);
            }
            else {
                // Get pointcloud from file
                std::ostringstream model_path;
                model_path << model_dir_ 
                        << model_name << "/visual_meshes/";
                std::string pcd_file_name = model_path.str() + "cloud.pcd";
                cloud.setPointCloud(pcd_file_name, default_view_point, model_scale);
            }

            // Preprocess the point cloud.
            preprocessPointCloud(cloud);

            // If the object is centered at the origin, reverse all surface normals.
            if (centered_at_origin_) {
            printf("Reversing normal directions ...\n");
            cloud.setNormals(cloud.getNormals() * (-1.0));
            }

            // Generate Candidates
            std::vector<std::unique_ptr<candidate::Hand>> grasps = detectGrasps(cloud);
            // Save grasps into msg
            if(grasps.size() < 1) {
                ROS_INFO("No Grasp Pose Found From GPD.");
            } 
            else {
                ROS_INFO("Grasp Pose Found");      
                // Save it into the msg
                for(int grasp_i = 0; grasp_i < grasps.size();++grasp_i) {
                    // Set Grasp Pose
                    Eigen::Vector3d bottom  = grasps[grasp_i]->getPosition();
                    Eigen::Matrix3d frame   = grasps[grasp_i]->getFrame();
                    float grasp_width = grasps[grasp_i]->getGraspWidth();
                    generate_msg(global_grasp_msg, 
                                model_name, model_scale, grasp_width,
                                frame, bottom,
                                object_frame_matrix,
                                object_position,
                                false);
                    if(flip_enable_) {
                        // generate another with symmetric over x-axis
                        Eigen::AngleAxis<double> flip(1.0*M_PI, Eigen::Vector3d(1.,0.,0.));
                        frame = frame * flip.toRotationMatrix();
                        generate_msg(global_grasp_msg, 
                                    model_name, model_scale, grasp_width,
                                    frame, bottom,
                                    object_frame_matrix,
                                    object_position,
                                    false);
                    }
                }
            }
        }
        // sort msg by prefer-direction
        std::cout << "Grasp SIZE: " << global_grasp_msg.grasp_poses.size() << std::endl;
        Eigen::Vector3d prefer_direction(0.0, 0.0, -1.0);
        sort_grasp(global_grasp_msg, prefer_direction);
        std::cout << "Grasp SIZE: " << global_grasp_msg.grasp_poses.size() << std::endl;
        // Add grasp_msg into res
        res.grasps.global_grasp_poses.push_back(global_grasp_msg);
    }
    return true;
}

void GraspDetector::generate_msg(grasp_srv::GlobalGraspPose& global_grasp_msg,
                                 const std::string model_name,
                                 const double model_scale,
                                 const double grasp_width,
                                 const Eigen::Matrix3d& frame_matrix, 
                                 const Eigen::Vector3d& bottom_vector,
                                 const Eigen::Matrix3d& object_frame_matrix,
                                 const Eigen::Vector3d& object_position,
                                 const bool enable_filter,
                                 const double relative_scale,
                                 const double comp_distance,
                                 const double pre_distance,
                                 const double angle_ratio) {
    // Scale the frame  
    Eigen::Matrix3d frame = frame_matrix;
    Eigen::Vector3d bottom = bottom_vector;

    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translate(bottom).rotate(frame).scale(relative_scale);
    Eigen::Matrix4d T_matrix = T.matrix();
    // update frame & bottom
    frame = T_matrix.block(0, 0, 3, 3);
    bottom = T_matrix.block(0, 3, 3, 1);
    
    Eigen::Vector3d raw_bottom = bottom;
    Eigen::Vector3d comp_vector(-comp_distance, 0.0, 0.0);
    bottom += (frame * comp_vector);
    Eigen::Vector3d pre_bottom = bottom;

    Eigen::Vector3d pre_vector(-pre_distance, 0.0, 0.0);
    pre_bottom += (frame * pre_vector);

    // Grasp Frame
    Eigen::Matrix3d grasp_frame = object_frame_matrix * frame;
    
    // Grasp Filter
    // Get Grasp orientation
    Eigen::Vector3d ori_vector(1.0, 0.0, 0.0);
    ori_vector = grasp_frame * ori_vector;  // world orientation

    if(enable_filter) {
        bool reachable = false;
        double grasp_z = ori_vector.z();
        if(grasp_z < 0) {
            reachable = true;
        }

        if(!reachable) return;  // Filtered unreachable pose
    }

    // Get Global Pose
    Eigen::Vector3d grasp_point = object_frame_matrix * bottom + object_position;
    Eigen::Vector3d pre_grasp_point = object_frame_matrix * pre_bottom + object_position;

    Eigen::Quaternion<double> grasp_frame_quat(grasp_frame);
    Eigen::Quaternion<double> frame_quat(frame);
    geometry_msgs::Pose grasp_pose;
    geometry_msgs::Pose pre_grasp_pose;
    geometry_msgs::Pose local_pose;
   
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

    // local_pose
    local_pose.position.x = raw_bottom.x();
    local_pose.position.y = raw_bottom.y();
    local_pose.position.z = raw_bottom.z();
    local_pose.orientation.x = frame_quat.x();
    local_pose.orientation.y = frame_quat.y();
    local_pose.orientation.z = frame_quat.z();
    local_pose.orientation.w = frame_quat.w();

    global_grasp_msg.grasp_poses.push_back(grasp_pose);
    global_grasp_msg.pre_grasp_poses.push_back(pre_grasp_pose);
    global_grasp_msg.local_poses.push_back(local_pose);

    // Set grasp width
    global_grasp_msg.grasp_widths.push_back(grasp_width * relative_scale);
    global_grasp_msg.model_names.push_back(model_name);
    global_grasp_msg.scales.push_back(model_scale);
}

// map cmp
bool cmp(const std::pair<int, double> a, 
         const std::pair<int, double> b) { 
    return a.second > b.second; 
} 

void GraspDetector::sort_grasp(grasp_srv::GlobalGraspPose& global_grasp_msg,
                               const Eigen::Vector3d& prefer_direction) {
    // establish a sorting_index
    std::vector<std::pair<int, double> > distance_by_id; 
    for(int i = 0; i < global_grasp_msg.grasp_poses.size(); ++i) {
        // local pose
        geometry_msgs::Pose grasp_pose = global_grasp_msg.grasp_poses[i];
        // get frame matrix
        Eigen::Quaternion<double> grasp_frame_quat(
            grasp_pose.orientation.w,
            grasp_pose.orientation.x,
            grasp_pose.orientation.y,
            grasp_pose.orientation.z);
        Eigen::Matrix3d grasp_frame = grasp_frame_quat.matrix();
        // orientation
        Eigen::Vector3d ori_vector(1.0, 0.0, 0.0);
        ori_vector = grasp_frame * ori_vector;  // world orientation
        double overlap = ori_vector.dot(prefer_direction);
        distance_by_id.push_back(std::pair<int, double>(i, overlap));
    }
    // sort by overlap
    std::stable_sort(distance_by_id.begin(), distance_by_id.end(), cmp);

    // rearrange order
    grasp_srv::GlobalGraspPose sorted_grasp_msg;
    for(auto p : distance_by_id) {
        int grasp_id = p.first;
        sorted_grasp_msg.grasp_poses.push_back(
            global_grasp_msg.grasp_poses[grasp_id]);
        sorted_grasp_msg.pre_grasp_poses.push_back(
            global_grasp_msg.pre_grasp_poses[grasp_id]);
        sorted_grasp_msg.local_poses.push_back(
            global_grasp_msg.local_poses[grasp_id]);

        // Set grasp width
        sorted_grasp_msg.grasp_widths.push_back(
            global_grasp_msg.grasp_widths[grasp_id]);
        sorted_grasp_msg.model_names.push_back(
            global_grasp_msg.model_names[grasp_id]);
        sorted_grasp_msg.scales.push_back(
            global_grasp_msg.scales[grasp_id]);
        std::cout << p.second << std::endl;
    }

    // re-assign
    global_grasp_msg = sorted_grasp_msg;
    return;
}

}  // namespace gpd
