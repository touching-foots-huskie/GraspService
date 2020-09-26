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
    flowLayout = new QVBoxLayout;
    // QWidget* scrollAreaContent = new QWidget;
    // scrollAreaContent->setLayout(flowLayout);
    // QScrollArea* scrollArea = new QScrollArea;
    // scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    // scrollArea->setWidgetResizable(true);
    // scrollArea->setWidget(scrollAreaContent);
    // visual_layout->addWidget(scrollArea);
    visual_layout->addLayout(flowLayout);

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

void GraspVisualizer::read_image(int grasp_id) {
    QHBoxLayout* grasp_render;
    std::string image1_filename = grasp_path_ 
                                + model_name_ 
                                + "/" + std::to_string(grasp_id)
                                + "_front.jpg";
    std::cout << image1_filename << std::endl;
    // QString qfilename1(image1_filename.c_str());  
    // QImage qimage1 = QImage(qfilename1);   
    // QPixmap pmap = QPixmap::fromImage(qimage1); // load pixmap
    QLabel* qlabel1 = new QLabel("Test");
    // qlabel1->setPixmap(pmap);
    grasp_render->addWidget(qlabel1);
    flowLayout->addLayout(grasp_render);
}

// SLOT
void GraspVisualizer::save() {}

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

    // go over all grasps
    for(int i = 0; i < grasps_.global_grasp_poses[0].model_names.size(); ++i) {
        render_grasp(i);
        ros::Duration(3.0).sleep();
        read_image(i);
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
    model_name_ = object_datas[model_id_];
    QString qname = QString::fromStdString(model_name_);
    model_name_label->setText(qname);
}

void GraspVisualizer::update_graspmode() {
    QString graspmode_string = grasp_mode_text->text();
    grasp_mode_ = graspmode_string.toStdString();
}