#ifndef SCENE_VISUALIZER_H
#define SCENE_VISUALIZER_H

#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <QLabel>
#include <QString>
#include <QLineEdit>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <iostream>
#include <istream>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include "grasp_srv/ObjectPoses.h"

// Math
#include <cmath>

// Json
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace rviz
{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "SceneVisualizer" implements the top level widget for this example.
class SceneVisualizer: public QWidget
{
Q_OBJECT
public:
    SceneVisualizer(std::string data_path, QWidget* parent = 0);
    virtual ~SceneVisualizer();

private Q_SLOTS:
    void nextGrasp();
    void prevGrasp();
    void updateModelName();
    void updateSceneName();
    void startGen();
    void save();
private:
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* robot_;
    rviz::Display* scene_;
    rviz::Display* axis_;

    // Edit Content
    QLineEdit* scene_name_text_;
    QComboBox* model_name_list_;

    // Push Button
    QPushButton* bt1_;
    QPushButton* bt2_;
    QPushButton* bt3_;
    QPushButton* bt4_;
    
    // publishing data
    int grasp_id_;
    std::string model_name_;
    std::string scene_name_;

    // Data Path
    std::string data_path_;

    // Node
    ros::NodeHandle nh_;
    // Ros Publisher
    ros::Publisher grasp_pub_;
    ros::Publisher scene_name_pub_;
    ros::Publisher model_name_pub_;
    ros::Publisher save_signal_pub_;
    // Rviz Config
    std::string frame_name_;
};
#endif // SCENE_VISUALIZER_H