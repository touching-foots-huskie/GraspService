#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "visualizer.h"


// Constructor
Visualizer::Visualizer(QWidget* parent) : QWidget(parent) {
    grasp_id_ = 0;
    frame_name_ = "base";
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
    main_layout->addLayout(visual_layout, 0, 0, 4, 6);
    main_layout->addLayout(control_layout, 4, 0, 1, 4);
    setLayout(main_layout);

    // Image Label
    color_image_label_ = new QLabel();
    color_image_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    // Main panel
    render_panel_ = new rviz::RenderPanel();
    visual_layout->addWidget(render_panel_, 0, 0, 4, 4);
    visual_layout->addWidget(color_image_label_, 2, 4, 2, 2);
    // Size control
    for(int c = 0; c < 6; c++) {
        visual_layout->setColumnMinimumWidth(c, 160);
    }
    for(int r = 0; r < 4; r++) {
        visual_layout->setRowMinimumHeight(r, 160);
    }

    // Control Button
    QPushButton* next_button = new QPushButton("Next", this);
    QPushButton* prev_button = new QPushButton("Prev", this);
    connect(next_button, SIGNAL (released()), this, SLOT (nextGrasp()));
    connect(prev_button, SIGNAL (released()), this, SLOT (prevGrasp()));

    control_layout->addWidget(prev_button, 0, 0, 1, 2);
    control_layout->addWidget(next_button, 0, 2, 1, 2);

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