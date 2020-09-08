#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>

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

private:
    void readColorImage(std::string img_path);
    void showImage();

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* robot_;
    rviz::Display* object_;
    // Label
    QLabel* color_image_label_;

    int grasp_id_;
    std::string data_path_;
    std::string frame_name_;

    // Ros Publisher
    ros::NodeHandle nh_;
    ros::Publisher grasp_pub_;
};
// END_TUTORIAL
#endif // VISUALIZER_H