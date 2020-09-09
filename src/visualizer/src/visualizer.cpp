#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "visualizer.h"


// Constructor
Visualizer::Visualizer(QWidget* parent) : QWidget(parent) {
    grasp_id_ = 0;
    frame_name_ = "world";
    object_name_ = "a_cups";
    std::fill(object_pose_, object_pose_ + 7, 0.0f);
    object_pose_[6] = 1.0f;
    object_scale_ = 1.0f;
    // Object Pub
    objectpose_pub_ = nh_.advertise<grasp_srv::ObjectPoses>("object_poses", 1);
    // Data Path
    data_path_ = "/home/harvey/Data/Reconstruction";
    // Initialize publisher
    grasp_pub_ = nh_.advertise<std_msgs::Int32>("grasp_index", 1);
    // QT
    this->setWindowTitle("Reconstruction Visualizer");
    // Layout
    QGridLayout* main_layout = new QGridLayout;
    QGridLayout* visual_layout = new QGridLayout;
    QGridLayout* control_layout = new QGridLayout;
    main_layout->addLayout(visual_layout, 0, 0, 8, 12);
    main_layout->addLayout(control_layout, 8, 0, 2, 8);
    setLayout(main_layout);

    // Image Label
    color_image_label_ = new QLabel();
    color_image_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    // Main panel
    render_panel_ = new rviz::RenderPanel();
    visual_layout->addWidget(render_panel_, 0, 0, 8, 8);
    visual_layout->addWidget(color_image_label_, 4, 8, 4, 4);
    // Size control
    for(int c = 0; c < 12; c++) {
        visual_layout->setColumnMinimumWidth(c, 80);
    }
    for(int r = 0; r < 8; r++) {
        visual_layout->setRowMinimumHeight(r, 80);
    }

    // Control Button
    QPushButton* next_button = new QPushButton("Next", this);
    QPushButton* prev_button = new QPushButton("Prev", this);
    connect(next_button, SIGNAL (released()), this, SLOT (nextGrasp()));
    connect(prev_button, SIGNAL (released()), this, SLOT (prevGrasp()));

    control_layout->addWidget(prev_button, 0, 0, 2, 4);
    control_layout->addWidget(next_button, 0, 4, 2, 4);

    // Start Button
    QPushButton* start_button = new QPushButton("Start", this);
    connect(start_button, SIGNAL (released()), this, SLOT (startGen()));
    visual_layout->addWidget(start_button, 3, 8, 1, 4);

    // ModelName Text
    modelname_text_ = new QLineEdit("a_cups", this);
    connect(modelname_text_, SIGNAL (editingFinished()), this, SLOT (updateModelName()));
    visual_layout->addWidget(modelname_text_, 2, 10, 1, 2);
    QLabel* model_name_label = new QLabel();
    QString model_name_text = QString::fromStdString("Model Name");
    model_name_label->setText(model_name_text);
    visual_layout->addWidget(model_name_label, 2, 8, 1, 2);

    // ModelPose Text
    modelpose_text_ = new QLineEdit("0 0 0 0 0 0 1", this);
    connect(modelpose_text_, SIGNAL (editingFinished()), this, SLOT (updateModelPose()));
    visual_layout->addWidget(modelpose_text_, 1, 10, 1, 2);
    QLabel* model_pose_label = new QLabel();
    QString model_pose_text = QString::fromStdString("Model Pose");
    model_pose_label->setText(model_pose_text);
    visual_layout->addWidget(model_pose_label, 1, 8, 1, 2);

    // ModelScale Text
    modelscale_text_ = new QLineEdit("1.0", this);
    connect(modelscale_text_, SIGNAL (editingFinished()), this, SLOT (updateModelScale()));
    visual_layout->addWidget(modelscale_text_, 0, 10, 1, 2);
    QLabel* model_scale_label = new QLabel();
    QString model_scale_text = QString::fromStdString("Model Scale");
    model_scale_label->setText(model_scale_text);
    visual_layout->addWidget(model_scale_label, 0, 8, 1, 2);

    // Manager initialization 
    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();
    manager_->startUpdate();

    manager_->setFixedFrame(QString(frame_name_.c_str()));
    
    // Grid display
    grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT(grid_ != NULL);
    grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp("Color")->setValue(QColor(Qt::white));
    grid_->subProp("Line Style")->subProp("Line Width")->setValue(0.01f);
    grid_->subProp("Cell Size")->setValue(1.0f);

    // Robot Display
    robot_ = manager_->
        createDisplay("rviz/RobotModel", "robot", true);
    robot_->subProp("Robot Description")->setValue("robot_description");

    // Object Display
    object_ = manager_->
        createDisplay("rviz/PointCloud2", "object", true);
    object_->subProp("Topic")->setValue("/object");

    // Axis Dispaly
    axis_ =  manager_->
        createDisplay("rviz/Axes", "world_axis", true);
    axis_->subProp("Reference Frame")->setValue("world");
    axis_->subProp("Length")->setValue(0.5f);
    axis_->subProp("Radius")->setValue(0.01f);
    showImage();
}

// Destructor
Visualizer::~Visualizer() {
    delete manager_;
    delete color_image_label_;
    delete color_image_label_;
}

// SLOT
void Visualizer::nextGrasp() {
    grasp_id_++;
    showImage();
    // Publish frame_index
    std_msgs::Int32 msg;
    msg.data = grasp_id_;
    grasp_pub_.publish(msg);
}

void Visualizer::prevGrasp() {
    grasp_id_--;
    showImage();
    // Publish frame_index
    std_msgs::Int32 msg;
    msg.data = grasp_id_;
    grasp_pub_.publish(msg);
}

void Visualizer::startGen() {
    // Set Msg
    ros::Rate rate(10);
    grasp_srv::ObjectPoses msg;
    geometry_msgs::Pose object_pose;
    object_pose.position.x = object_pose_[0];
    object_pose.position.y = object_pose_[1];
    object_pose.position.z = object_pose_[2];
    object_pose.orientation.x = object_pose_[3];
    object_pose.orientation.y = object_pose_[4];
    object_pose.orientation.z = object_pose_[5];
    object_pose.orientation.w = object_pose_[6];
    msg.object_names.push_back(object_name_);
    msg.object_scales.push_back(object_scale_);
    msg.object_poses.push_back(object_pose);
    objectpose_pub_.publish(msg);
    rate.sleep();
    
    std::cout << "Object Msg published" << std::endl;
}

void Visualizer::updateModelName() {
    QString model_name = modelname_text_->text();
    object_name_ = model_name.toStdString();
    std::cout << "Model Name Updated" << std::endl;
}

void Visualizer::updateModelPose() {
    QString model_pose = modelpose_text_->text();
    std::string pose_string = model_pose.toStdString();
    // Parse String
    std::istringstream iss(pose_string);
    std::vector<std::string> results((std::istream_iterator<std::string>(iss)),
		                      std::istream_iterator<std::string>());

    for(int i = 0; i < 7; ++i) {
        double pvalue = std::stod(results[i]);
        object_pose_[i] = pvalue;
    }
    
    // Normalize
    double norm_p = 0.0;
    for(int i = 0; i < 4; ++i) {
        norm_p += pow(object_pose_[i+3], 2); 
    }
    norm_p = std::sqrt(norm_p);

    for(int i = 0; i < 4; ++i) {
        object_pose_[i+3] = object_pose_[i+3]/norm_p;
    }

    std::ostringstream ss;
    for(int i = 0; i < 7; ++i) {
        ss << object_pose_[i];
        if(i < 6) {
            ss << " ";
        }
    }
    // Reset String
    std::string pose_text(ss.str());
    QString qpose_text = QString::fromStdString(pose_text);
    modelpose_text_->setText(qpose_text);
    std::cout << "Model Pose Updated" << std::endl;
}

void Visualizer::updateModelScale() {
    QString model_scale = modelscale_text_->text();
    std::string scale_string = model_scale.toStdString();
    object_scale_ = std::stod(scale_string);
    std::cout << "Model Scale Updated" << std::endl;
}

void Visualizer::readColorImage(std::string img_path) {
    QImage image(img_path.c_str());
    QPixmap p = QPixmap::fromImage(image); // load pixmap
    // get label dimensions
    int w = color_image_label_->width();
    int h = color_image_label_->height();
    // TODO: there are still some bugs in the first frames
    std::cout << "Width: " << w << ", Height: " << h << std::endl;
    // set a scaled pixmap to a w x h window keeping its aspect ratio 
    color_image_label_->setPixmap(p.scaled(w, h, Qt::KeepAspectRatio));
}

void Visualizer::showImage() {
    std::string color_img_path = data_path_ + "/Color/Img_" + 
        std::to_string(grasp_id_) + ".PNG";
    readColorImage(color_img_path);
    // Log
    std::cout << "Img Path: " << color_img_path << std::endl;
}
