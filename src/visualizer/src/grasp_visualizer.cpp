#include "grasp_visualizer.h"


// Constructor
GraspVisualizer::GraspVisualizer(QWidget* parent) : QWidget(parent) {
    // data initialization
    model_id_ = 0;
    grasp_mode_ = "box";
    data_path_ = "/home/harvey/Data/Reconstruction";
    
    // initialize publisher
    model_id_pub_ = nh_.advertise<std_msgs::Int32>("model_id", 1);
    grasp_mode_pub_ = nh_.advertise<std_msgs::String>("grasp_mode", 1);
    save_signal_pub_ = nh_.advertise<std_msgs::Bool>("save_signal", 1);
    start_signal_pub_ = nh_.advertise<std_msgs::Bool>("start_signal", 1);
    
    // Title
    this->setWindowTitle("Grasp pose Visualizer");
    
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
    QLabel* grasp_mode_label = new QLabel("Grasp Mode: ", this);
    grasp_mode_text = new QLineEdit("box", this);
    choice_layout->addWidget(model_id_label);
    choice_layout->addWidget(model_id_text);
    choice_layout->addWidget(grasp_mode_label);
    choice_layout->addWidget(grasp_mode_text);

    // Visual Layout

    // Control Layout
    bt1_ = new QPushButton("Start", this);
    bt2_ = new QPushButton("Save", this);
    connect(bt1_, SIGNAL (released()), this, SLOT (start()));
    connect(bt2_, SIGNAL (released()), this, SLOT (save()));

    control_layout->addWidget(bt1_);
    control_layout->addWidget(bt2_);
}

// Destructor
GraspVisualizer::~GraspVisualizer() {   
}

// SLOT
GraspVisualizer::save() {}

GraspVisualizer::start() {}
