#ifndef GRASP_VISUALIZER_H
#define GRASP_VISUALIZER_H

// Qt
#include <QWidget>
// Label
#include <QPixmap>
#include <QImage>
#include <QLabel>
#include <QString>
#include <QLineEdit>
// Layout
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
// Button
#include <QPushButton>
#include <iostream>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include "grasp_srv/ObjectPoses.h"

// Math
#include <cmath>


namespace rviz
{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "GraspVisualizer" implements the top level widget for this example.
class GraspVisualizer: public QWidget
{
Q_OBJECT
public:
    GraspVisualizer(QWidget* parent = 0);
    virtual ~GraspVisualizer();

private Q_SLOTS:
    void save();
    void start();

private:
    void readColorImage(std::string img_path);
    void showImage();

    // publishing data
    std::string model_id_;
    std::string grasp_mode_;  // box, can, gpd, flat
    std::string data_path_;

    // Ros Publisher
    ros::NodeHandle nh_;
    ros::Publisher model_id_pub_;      // publish model name
    ros::Publisher grasp_mode_pub_;   // publish grasp model
    ros::Publisher save_signal_pub_;   // save signal
    ros::Publisher start_signal_pub_;  // start signal

    // bt
    QPushButton* bt1_;  // save
    QPushButton* bt2_;  // start

    // edit
    QLineEdit* model_id_text;
    QLineEdit* grasp_mode_text;


};
#endif // GRASP_VISUALIZER_H