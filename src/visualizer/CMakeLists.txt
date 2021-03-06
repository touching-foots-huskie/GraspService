cmake_minimum_required(VERSION 3.0.2)
project(grasp_visualizer)
find_package(catkin REQUIRED COMPONENTS rviz roscpp grasp_srv)
find_package(nlohmann_json 3.2.0 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
 CATKIN_DEPENDS 
 rviz
 roscpp 
 grasp_srv
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/visualizer.cpp
  src/main.cpp
)

add_executable(visualizer ${SRC_FILES})
target_link_libraries(visualizer ${QT_LIBRARIES} ${catkin_LIBRARIES})

set(SCENE_SRC_FILES
  src/scene_visualizer.cpp
  src/scene_main.cpp
)

add_executable(scene_visualizer ${SCENE_SRC_FILES})
target_link_libraries(scene_visualizer 
                      ${QT_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      nlohmann_json::nlohmann_json)

set(GRASP_SRC_FILES
  src/grasp_visualizer.cpp
  src/grasp_main.cpp
)

add_executable(grasp_visualizer ${GRASP_SRC_FILES})
target_link_libraries(grasp_visualizer 
                      ${QT_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      nlohmann_json::nlohmann_json)

## Install
install(TARGETS visualizer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS scene_visualizer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS grasp_visualizer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})