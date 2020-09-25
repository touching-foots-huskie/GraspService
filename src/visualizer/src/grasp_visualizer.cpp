#include "grasp_visualizer.h"


// Constructor
GraspVisualizer::GraspVisualizer(QWidget* parent) : QWidget(parent) {
    // data initialization
    model_id_ = "0";
    grasp_mode_ = "box";
    data_path_ = "/root/ocrtoc_materials";
    
    // initialize publisher
    model_id_pub_ = nh_.advertise<std_msgs::Int32>("model_id", 1);
    grasp_mode_pub_ = nh_.advertise<std_msgs::String>("grasp_mode", 1);
    save_signal_pub_ = nh_.advertise<std_msgs::Bool>("save_signal", 1);
    
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

    // Control Layout
    bt1_ = new QPushButton("Start", this);
    bt2_ = new QPushButton("Save", this);
    connect(bt1_, SIGNAL(released()), this, SLOT(start()));
    connect(bt2_, SIGNAL(released()), this, SLOT(save()));

    control_layout->addWidget(bt1_);
    control_layout->addWidget(bt2_);

    // initialize function
    update_modelname();
    update_graspmode();
}

// Destructor
GraspVisualizer::~GraspVisualizer() {   
}

// SLOT
void GraspVisualizer::save() {}

void GraspVisualizer::start() {
    grasp_srv::GraspGen srv;
    // set name
    srv.request.object_poses.object_names.push_back(model_name_);
    // set scale
    srv.request.object_poses.object_scales.push_back(1.0);
    // set grasp mode
    srv.request.object_poses.grasp_modes.push_back(grasp_mode_);
    // empty pointcloud
    sensor_msgs::PointCloud2 pointcloud;
    srv.request.object_poses.object_point_clouds.push_back(pointcloud);
    // upper pose
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.2;
    srv.request.object_poses.object_poses.push_back(pose);
    
    // call service
    if(client_.call(srv)) {
        ROS_INFO("Grasp Pose Generated");
    }
    else {
        ROS_ERROR("Failed to call service grasp_gen");
    }
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
    std::string model_name = object_datas[model_id_];
    QString qname = QString::fromStdString(model_name);
    model_name_label->setText(qname);
}

void GraspVisualizer::update_graspmode() {
    QString graspmode_string = grasp_mode_text->text();
    grasp_mode_ = graspmode_string.toStdString();
}