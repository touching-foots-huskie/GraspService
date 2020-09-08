// BEGIN_TUTORIAL

// The main() for this "myviz" example is very simple, it just
// initializes ROS, creates a QApplication, creates the top-level
// widget (of type "Visualizer"), shows it, and runs the Qt event loop.

#include <QApplication>
#include <ros/ros.h>
#include "visualizer.h"

// Load PCD file

int main(int argc, char **argv)
{
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "visualizer", ros::init_options::AnonymousName);
  }

  QApplication app(argc, argv);

  Visualizer* myviz = new Visualizer();
  myviz->show();

  app.exec();

  delete myviz;
}
