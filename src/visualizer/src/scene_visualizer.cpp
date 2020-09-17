#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "scene_visualizer.h"


// Constructor
SceneVisualizer::SceneVisualizer(std::string data_path, QWidget* parent) : QWidget(parent) {
    // Default setting
    frame_name_ = "world";
    grasp_id_ = 0;
    model_name_ = "a_cups";
    scene_name_ = "1-1";
    data_path_ = data_path;
    
    // Initialize publisher
    grasp_pub_      = nh_.advertise<std_msgs::Int32>("grasp_id", 1);
    scene_name_pub_ = nh_.advertise<std_msgs::String>("scene_name", 1);
    model_name_pub_ = nh_.advertise<std_msgs::String>("model_name", 1);
    save_signal_pub_ = nh_.advertise<std_msgs::Bool>("save_signal", 1);

    // QT
    this->setWindowTitle("Scene Visualizer");
    // Layout
    QGridLayout* main_layout = new QGridLayout;
    QGridLayout* visual_layout = new QGridLayout;
    QHBoxLayout* control_layout = new QHBoxLayout;

    main_layout->addLayout(visual_layout, 0, 0, 8, 8);
    main_layout->addLayout(control_layout, 8, 0, 2, 8);
    setLayout(main_layout);

    // Visual Layout
    render_panel_ = new rviz::RenderPanel();
    visual_layout->addWidget(render_panel_, 0, 0, 8, 8);

    // Size control
    for(int c = 0; c < 8; c++) {
        visual_layout->setColumnMinimumWidth(c, 80);
    }
    for(int r = 0; r < 8; r++) {
        visual_layout->setRowMinimumHeight(r, 80);
    }

    // Control Button
    QGroupBox *graspControl = new QGroupBox(tr("Grasp Control"));
    bt1_ = new QPushButton("Next", this);
    bt2_ = new QPushButton("Prev", this);
    bt4_ = new QPushButton("Save", this);
    connect(bt1_, SIGNAL (released()), this, SLOT (nextGrasp()));
    connect(bt2_, SIGNAL (released()), this, SLOT (prevGrasp()));
    connect(bt4_, SIGNAL (released()), this, SLOT (save()));
    QVBoxLayout *btbox = new QVBoxLayout;
    btbox->addWidget(bt1_);
    btbox->addWidget(bt2_);
    btbox->addWidget(bt4_);
    graspControl->setLayout(btbox);
    control_layout->addWidget(graspControl);

    // Name Area
    QGroupBox *sceneControl = new QGroupBox(tr("Scene Choice"));
    scene_name_text_ = new QLineEdit("1-1", this);
    model_name_list_ = new QComboBox(this);
    bt3_ = new QPushButton("Start", this);

    connect(scene_name_text_, SIGNAL(editingFinished()), this, SLOT(updateSceneName()));
    connect(model_name_list_, SIGNAL(activated(int)), this, SLOT(updateModelName()));
    connect(bt3_, SIGNAL (released()), this, SLOT (startGen()));
    
    QVBoxLayout *ttbox = new QVBoxLayout;
    ttbox->addWidget(scene_name_text_);
    ttbox->addWidget(model_name_list_);
    ttbox->addWidget(bt3_);
    sceneControl->setLayout(ttbox);
    control_layout->addWidget(sceneControl);

    // Set Stretch
    control_layout->setStretch(0, 10);
    control_layout->setStretch(1, 10);

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

    // Scene Display
    scene_ = manager_->
        createDisplay("rviz/PointCloud2", "scene", true);
    scene_->subProp("Topic")->setValue("/scene");
    scene_->subProp("Color Transformer")->setValue("RGB8");
    scene_->subProp("Style")->setValue("Flat Squares");

    // Axis Dispaly
    axis_ =  manager_->
        createDisplay("rviz/Axes", "world_axis", true);
    axis_->subProp("Reference Frame")->setValue("world");
    axis_->subProp("Length")->setValue(0.5f);
    axis_->subProp("Radius")->setValue(0.01f);

    // First Update
    updateSceneName();
    updateModelName();
}

// Destructor : delete newed ptr
SceneVisualizer::~SceneVisualizer() {
    delete manager_;
    delete render_panel_;
    delete grid_;
    delete robot_;
    delete scene_;
    delete axis_;
    delete scene_name_text_;
    delete model_name_list_;
    delete bt1_;
    delete bt2_;
    delete bt3_;
}

// SLOT
void SceneVisualizer::nextGrasp() {
    grasp_id_++;
    // Publish frame_index
    std_msgs::Int32 msg;
    msg.data = grasp_id_;
    grasp_pub_.publish(msg);
}

void SceneVisualizer::prevGrasp() {
    grasp_id_--;
    // Publish frame_index
    std_msgs::Int32 msg;
    msg.data = grasp_id_;
    grasp_pub_.publish(msg);
}

void SceneVisualizer::save() {
    std_msgs::Bool msg;
    msg.data = true;
    save_signal_pub_.publish(msg);
}

void SceneVisualizer::updateModelName() {
    QString model_name = model_name_list_->currentText();
    model_name_ = model_name.toStdString();
    std::cout << "Model Name Updated" << std::endl;
}

void SceneVisualizer::updateSceneName() {
    QString scene_name = scene_name_text_->text();
    scene_name_ = scene_name.toStdString();
    std::cout << "SceneName Updated" << std::endl;
    // Parse Json File to update Model Name
    std::string jsonfile_path = data_path_ + scene_name_ + "/object_data.json";
    std::ifstream json_file(jsonfile_path);
    json object_datas;
    json_file >> object_datas;
    // Go through data in json files
    model_name_list_->clear();
    for(auto i = 0; i < object_datas.size(); ++i) {
        std::string object_name = object_datas[i]["name"];
        QString qname = QString::fromStdString(object_name);
        model_name_list_->insertItem(model_name_list_->count()-1, qname);
    }
}

// Set everything in the msg
void SceneVisualizer::startGen() {
    // Send Msg
    std_msgs::String scene_msg;
    scene_msg.data = scene_name_;
    scene_name_pub_.publish(scene_msg);

    std_msgs::String model_msg;
    model_msg.data = model_name_;
    model_name_pub_.publish(model_msg);

    // Reset graspid
    grasp_id_ = 0;
    std_msgs::Int32 grasp_msg;
    grasp_msg.data = grasp_id_;
    grasp_pub_.publish(grasp_msg);

    ROS_INFO("New Scene Start!");
}

