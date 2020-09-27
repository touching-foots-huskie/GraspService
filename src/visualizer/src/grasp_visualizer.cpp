#include "grasp_visualizer.h"


// Constructor
GraspVisualizer::GraspVisualizer(QWidget* parent) : QWidget(parent) {
    // data initialization
    model_id_ = "0";
    model_name_ = "";
    model_scale_ = 0.5;
    grasp_mode_ = "box";
    data_path_ = "/root/ocrtoc_materials";
    grasp_path_ = "/root/GraspService/src/grasp_srv/grasp_data/";

    // initialize publisher
    grasp_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("grasp_pose", 1);
    // object publisher
    object_name_pub_ = nh_.advertise<std_msgs::String>("object_name", 1);
    object_scale_pub_ = nh_.advertise<std_msgs::Float64>("object_scale", 1);
    object_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("object_pose", 1);

    grasp_id_pub_ = nh_.advertise<std_msgs::Int32>("grasp_id", 1);

    // initialize client
    client_ = nh_.serviceClient<grasp_srv::GraspGen>("grasp_gen");
    
    // Title
    this->setWindowTitle("Grasp pose Selector");
    
    // Layout
    QVBoxLayout* main_layout = new QVBoxLayout;
    QHBoxLayout* choice_layout = new QHBoxLayout;
    QVBoxLayout* visual_layout = new QVBoxLayout;
    QHBoxLayout* control_layout = new QHBoxLayout;

    main_layout->addLayout(choice_layout);
    main_layout->addLayout(visual_layout);
    main_layout->addLayout(control_layout);

    setLayout(main_layout);

    // Choice Layout
    QLabel* model_id_label = new QLabel("Model ID: ", this);
    model_id_text = new QLineEdit("0", this);
    connect(model_id_text, SIGNAL(editingFinished()), this, SLOT(update_modelname()));

    model_name_label = new QLabel("", this);
    QLabel* grasp_mode_label = new QLabel("Grasp Mode: ", this);
    grasp_mode_text = new QLineEdit("box", this);
    connect(grasp_mode_text, SIGNAL(editingFinished()), this, SLOT(update_graspmode()));

    choice_layout->addWidget(model_id_label);
    choice_layout->addWidget(model_id_text);
    choice_layout->addWidget(model_name_label);
    choice_layout->addWidget(grasp_mode_label);
    choice_layout->addWidget(grasp_mode_text);

    // Visual Layout
    QHBoxLayout* renderLayout = new QHBoxLayout;
    upper_view = new QLabel("Upper View", this);
    front_view = new QLabel("Front View", this);
    right_view = new QLabel("Right View", this);
    renderLayout->addWidget(upper_view);
    renderLayout->addWidget(front_view);
    renderLayout->addWidget(right_view);
    visual_layout->addLayout(renderLayout);
    // test
    list_area = new QListWidget(this);
    connect(list_area, SIGNAL(itemActivated(QListWidgetItem*)), this, SLOT(read_image(QListWidgetItem*)));
    visual_layout->addWidget(list_area);

    // Control Layout
    bt1_ = new QPushButton("Start", this);
    bt2_ = new QPushButton("Save", this);
    bt3_ = new QPushButton("Show", this);
    connect(bt1_, SIGNAL(released()), this, SLOT(start()));
    connect(bt2_, SIGNAL(released()), this, SLOT(remove_grasp()));
    connect(bt3_, SIGNAL(released()), this, SLOT(render()));

    control_layout->addWidget(bt1_);
    control_layout->addWidget(bt2_);
    control_layout->addWidget(bt3_);


    // initialize function
    update_modelname();
    update_graspmode();
}

// Destructor
GraspVisualizer::~GraspVisualizer() {   
}

// render and save image
void GraspVisualizer::render_grasp(int grasp_id) {
    // publish pose | to render pose
    if(grasps_.global_grasp_poses[0].model_names.size() == 0) return;
    geometry_msgs::Pose grasp_pose = 
        grasps_.global_grasp_poses[0].grasp_poses[grasp_id];

    grasp_pose_pub_.publish(grasp_pose);

    // publish model name | to render object
    std_msgs::String object_name_msg;
    object_name_msg.data = model_name_;
    object_name_pub_.publish(object_name_msg);

    std_msgs::Float64 object_scale_msg;
    object_scale_msg.data = model_scale_;
    object_scale_pub_.publish(object_scale_msg);

    object_pose_pub_.publish(object_poses_.object_poses[0]);

    // publish id
    std_msgs::Int32 id_msg;
    id_msg.data = grasp_id;
    grasp_id_pub_.publish(id_msg);  // Id 
}

// SLOT
void GraspVisualizer::start() {
    grasp_srv::GraspGen srv;
    grasp_srv::ObjectPoses object_poses_msg;
    // set name
    object_poses_msg.object_names.push_back(model_name_);
    // set scale
    object_poses_msg.object_scales.push_back(model_scale_);
    // set grasp mode
    object_poses_msg.grasp_modes.push_back(grasp_mode_);
    // empty pointcloud
    sensor_msgs::PointCloud2 pointcloud;
    object_poses_msg.object_point_clouds.push_back(pointcloud);
    // upper pose
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.2;
    object_poses_msg.object_poses.push_back(pose);
    
    srv.request.object_poses = object_poses_msg;
    object_poses_ = object_poses_msg;  // reassign msg

    // call service
    if(client_.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
        grasps_ = srv.response.grasps;  // update grasps
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
    }

    // set json file
    poses_json_.clear();
    int num_grasps = grasps_.global_grasp_poses[0].local_poses.size();
    for(int i = 0; i < num_grasps; ++i) {
        geometry_msgs::Pose local_pose = 
            grasps_.global_grasp_poses[0].local_poses[i];
        double scale = grasps_.global_grasp_poses[0].scales[i];
        double grasp_width = grasps_.global_grasp_poses[0].grasp_widths[i];
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
        poses_json_[std::to_string(i)] = pose_array;
    }
    // save json file
    std::string pose_filename = grasp_path_ 
                              + model_name_ 
                              + "/"
                              + "pose.json";
    std::ofstream out_file(pose_filename);
    out_file << poses_json_;
    out_file.close();

    // render grasps
    for(int i = 0; i < num_grasps; ++i) {
        render_grasp(i);
        ros::Duration(3.0).sleep();
    }
}

void GraspVisualizer::render() {
    std::string image_dir = grasp_path_ + model_name_;
    // iterate over directory
    std::vector<int> existing_id;
    recursive_directory_iterator end;
    for (recursive_directory_iterator it(image_dir); it != end; ++it) {
        std::string filename = it->path().filename().string();
        if((filename == "type.json") || (filename == "pose.json"))
            continue;
        std::string delimiter = "_";
        std::string token = filename.substr(0, filename.find(delimiter)); 
        int grasp_id = std::stoi(token);  
        bool exists = false;
        for(auto d : existing_id) {
            if(d == grasp_id) exists = true; 
        }      

        if(!exists) {
            existing_id.push_back(grasp_id);
        }              
    }

    // sort
    list_area->clear();  // clear list
    std::sort(existing_id.begin(), existing_id.end(), std::greater<int>());
    for(auto grasp_id : existing_id) {
        QListWidgetItem *item = new QListWidgetItem;
        std::string id_string = std::to_string(grasp_id);
        id_string = "ID: " + id_string;
        item->setCheckState(Qt::Unchecked);
        item->setText(QString(id_string.c_str()));
        list_area->insertItem(0, item);
    }

    // read json
    poses_json_.clear();
    std::string grasp_path = grasp_path_ + model_name_;
    std::string pose_filename = grasp_path + "/pose.json";
    std::ifstream json_file(pose_filename);
    json_file >> poses_json_;
    json_file.close();
}

void GraspVisualizer::update_modelname() {
    // update id
    QString model_id_string = model_id_text->text();
    model_id_ = model_id_string.toStdString();

    // get model name
    std::string jsonfile_path = data_path_ + "/object_id.json";
    std::ifstream json_file(jsonfile_path);
    json object_datas;
    json_file >> object_datas;
    model_name_ = object_datas[model_id_];
    QString qname = QString::fromStdString(model_name_);
    model_name_label->setText(qname);
}

void GraspVisualizer::update_graspmode() {
    QString graspmode_string = grasp_mode_text->text();
    grasp_mode_ = graspmode_string.toStdString();
    // update type json
    json type_json;
    std::string type_filename = grasp_path_ 
                              + model_name_ 
                              + "/"
                              + "type.json";
    // read json
    std::ifstream in_file(type_filename);
    in_file >> type_json;
    in_file.close();

    type_json["type"] = grasp_mode_;
    
    // write json
    std::ofstream out_file(type_filename);
    out_file << type_json;
    out_file.close();
}

void GraspVisualizer::read_image(QListWidgetItem *item) {
    std::string id_string = item->text().toStdString();
    std::string delimiter = ":";
    std::string token = id_string.substr(id_string.find(delimiter)+1, id_string.size()); 
    // remove space
    std::string::iterator end_pos = std::remove(token.begin(), token.end(), ' ');
    token.erase(end_pos, token.end());
    int grasp_id = std::stoi(token); 
    // QHBoxLayout* grasp_render;
    std::string image1_filename = grasp_path_ 
                                + model_name_ 
                                + "/" + std::to_string(grasp_id)
                                + "_upper.jpg";
    // Set upper view
    QString qfilename1(image1_filename.c_str());  
    QImage qimage1 = QImage(qfilename1);  
    QImage resize_image1 = qimage1.scaled(320, 240, Qt::KeepAspectRatio); 
    QPixmap pmap1 = QPixmap::fromImage(resize_image1); // load pixmap
    upper_view->setPixmap(pmap1);

    // Front view
    std::string image2_filename = grasp_path_ 
                                + model_name_ 
                                + "/" + std::to_string(grasp_id)
                                + "_front.jpg";
    QString qfilename2(image2_filename.c_str());  
    QImage qimage2 = QImage(qfilename2);  
    QImage resize_image2 = qimage2.scaled(320, 240, Qt::KeepAspectRatio); 
    QPixmap pmap2 = QPixmap::fromImage(resize_image2); // load pixmap
    front_view->setPixmap(pmap2);

    // Right view
    std::string image3_filename = grasp_path_ 
                                + model_name_ 
                                + "/" + std::to_string(grasp_id)
                                + "_right.jpg";
    QString qfilename3(image3_filename.c_str());  
    QImage qimage3 = QImage(qfilename3);  
    QImage resize_image3 = qimage3.scaled(320, 240, Qt::KeepAspectRatio); 
    QPixmap pmap3 = QPixmap::fromImage(resize_image3); // load pixmap
    right_view->setPixmap(pmap3);
}

void GraspVisualizer::remove_grasp() {
    // get all select 
    int item_num = list_area->count();
    for(int i = 0; i < item_num; ++i) {
        QListWidgetItem* item = list_area->item(i);

        if(item->checkState() != Qt::Checked) continue;

        std::string id_string = item->text().toStdString();
        std::string delimiter = ":";
        std::string token = id_string.substr(id_string.find(delimiter)+1, id_string.size()); 
        
        // remove space
        std::string::iterator end_pos = std::remove(token.begin(), token.end(), ' ');
        token.erase(end_pos, token.end());
        int grasp_id = std::stoi(token); 
        poses_json_.erase(std::to_string(grasp_id));

        // remove images
        std::string image1_filename = grasp_path_ 
                                    + model_name_ 
                                    + "/" + std::to_string(grasp_id)
                                    + "_upper.jpg";
        std::string image2_filename = grasp_path_ 
                                    + model_name_ 
                                    + "/" + std::to_string(grasp_id)
                                    + "_front.jpg";
        std::string image3_filename = grasp_path_ 
                                    + model_name_ 
                                    + "/" + std::to_string(grasp_id)
                                    + "_right.jpg";
        remove(image1_filename);
        remove(image2_filename);
        remove(image3_filename);
    }

    // save back
    std::string pose_filename = grasp_path_ 
                              + model_name_ 
                              + "/"
                              + "pose.json";
    std::ofstream out_file(pose_filename);
    out_file << poses_json_;
    out_file.close();

    // call render
    render();
}