// BEGIN_TUTORIAL

// The main() for this "myviz" example is very simple, it just
// initializes ROS, creates a QApplication, creates the top-level
// widget (of type "Visualizer"), shows it, and runs the Qt event loop.

#include <QApplication>
#include <ros/ros.h>
#include "scene_visualizer.h"


int main(int argc, char **argv)
{
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "visualizer", ros::init_options::AnonymousName);
  }

  QApplication app(argc, argv);
  std::string data_path(argv[1]);
  SceneVisualizer* myviz = new SceneVisualizer(data_path);
  
  myviz->show();

  app.exec();

  delete myviz;
}
