#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <QLabel>
#include <QString>
#include <QLineEdit>
#include <QGridLayout>
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
// Class "Visualizer" implements the top level widget for this example.
class Visualizer: public QWidget
{
Q_OBJECT
public:
    Visualizer(QWidget* parent = 0);
    virtual ~Visualizer();

private Q_SLOTS:
    void nextGrasp();
    void prevGrasp();
    void startGen();
    void updateModelName();
    void updateModelPose();
    void updateModelScale();

private:
    void readColorImage(std::string img_path);
    void showImage();

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* robot_;
    rviz::Display* object_;
    rviz::Display* axis_;
    // Label
    QLabel* color_image_label_;
    QLineEdit* modelname_text_;
    QLineEdit* modelpose_text_;
    QLineEdit* modelscale_text_;

    // publishing data
    int grasp_id_;
    std::string object_name_;
    double object_pose_[7];
    double object_scale_;

    std::string data_path_;
    std::string frame_name_;

    // Ros Publisher
    ros::NodeHandle nh_;
    ros::Publisher grasp_pub_;
    ros::Publisher objectpose_pub_;
};
// END_TUTORIAL
#endif // VISUALIZER_H