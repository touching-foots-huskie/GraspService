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
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
// msg & srv
#include "grasp_srv/ObjectPoses.h"
#include "grasp_srv/GraspGen.h"
#include "grasp_srv/Grasps.h"

// Math
#include <cmath>

// Json
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// stream
#include <istream>
#include <fstream>


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
    // render and save image
    void render_grasp(int grasp_id);

private Q_SLOTS:
    void save();
    void start();
    void update_modelname();
    void update_graspmode();

private:
    // publishing data
    std::string model_id_;
    std::string grasp_mode_;  // box, can, gpd, flat
    std::string data_path_;   // josn file for object id
    std::string model_name_; 
    double model_scale_;

    // grasps data
    grasp_srv::Grasps grasps_;
    grasp_srv::ObjectPoses object_poses_;

    // Ros Publisher
    ros::NodeHandle nh_;
    ros::Publisher grasp_pose_pub_;   // grasp pose
    ros::Publisher object_name_pub_; // object publisher
    ros::Publisher object_scale_pub_;
    ros::Publisher object_pose_pub_;
    ros::Publisher grasp_id_pub_;
    // client
    ros::ServiceClient client_;

    // bt
    QPushButton* bt1_;  // save
    QPushButton* bt2_;  // start

    // edit
    QLineEdit* model_id_text;
    QLineEdit* grasp_mode_text;

    // showing label
    QLabel* model_name_label;
};
#endif // GRASP_VISUALIZER_H