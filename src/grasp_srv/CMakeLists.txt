cmake_minimum_required(VERSION 3.0.2)
project(grasp_srv)

# Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS}) 

# ROS include
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# complier optimization flag
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare")

# catkin CMD
add_message_files(
  FILES
  Label.msg
  Configuration1D.msg
  GraspPose.msg
  Grasp.msg 
  Grasps.msg
  ObjectPoses.msg
  GlobalGraspPose.msg
)

add_service_files(
  FILES
  GraspGen.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
 roscpp 
 rospy
 rosbag
 std_msgs 
 geometry_msgs 
 sensor_msgs
 message_runtime 
 pcl_ros
 pcl_conversions
)

set(GRASP "gpg")
# Grasp Library
add_library(${GRASP}_grasp_candidates_generator SHARED src/${GRASP}/candidates_generator.cpp)

add_library(${GRASP}_antipodal src/${GRASP}/antipodal.cpp)
add_library(${GRASP}_cloud_camera src/${GRASP}/cloud_camera.cpp)
add_library(${GRASP}_config_file src/${GRASP}/config_file.cpp)
add_library(${GRASP}_eigen_utils src/${GRASP}/eigen_utils.cpp)
add_library(${GRASP}_finger_hand src/${GRASP}/finger_hand.cpp)
add_library(${GRASP}_frame_estimator src/${GRASP}/frame_estimator.cpp)
add_library(${GRASP}_grasp src/${GRASP}/grasp.cpp)
add_library(${GRASP}_grasp_set src/${GRASP}/grasp_set.cpp)
add_library(${GRASP}_hand_search src/${GRASP}/hand_search.cpp)
add_library(${GRASP}_local_frame src/${GRASP}/local_frame.cpp)
add_library(${GRASP}_plot src/${GRASP}/plot.cpp)
add_library(${GRASP}_point_list src/${GRASP}/point_list.cpp)

# This executable is for testing the shared library
add_executable(${GRASP}_generate_candidates src/generate_candidates.cpp)
target_link_libraries(${GRASP}_generate_candidates 
                      ${GRASP}_config_file 
                      ${GRASP}_grasp_candidates_generator)

# Linking for libraries
target_link_libraries(${GRASP}_antipodal 
                      ${GRASP}_point_list)

target_link_libraries(${GRASP}_cloud_camera 
                      ${GRASP}_eigen_utils 
                      ${PCL_LIBRARIES})

target_link_libraries(${GRASP}_eigen_utils 
                      ${EIGEN_LIBRARIES})

target_link_libraries(${GRASP}_frame_estimator 
                      ${GRASP}_cloud_camera 
                      ${GRASP}_local_frame)

target_link_libraries(${GRASP}_grasp 
                      ${GRASP}_finger_hand)

target_link_libraries(${GRASP}_grasp_set 
                      ${GRASP}_antipodal 
                      ${GRASP}_grasp 
                      ${GRASP}_local_frame 
                      ${GRASP}_point_list)

target_link_libraries(${GRASP}_hand_search 
                      ${GRASP}_antipodal 
                      ${GRASP}_cloud_camera  
                      ${GRASP}_frame_estimator 
                      ${GRASP}_grasp_set 
                      ${GRASP}_plot)

target_link_libraries(${GRASP}_local_frame 
                      ${PCL_LIBRARIES})

target_link_libraries(${GRASP}_plot 
                      ${GRASP}_cloud_camera 
                      ${GRASP}_grasp_set 
                      ${GRASP}_local_frame)

target_link_libraries(${GRASP}_grasp_candidates_generator 
                      ${GRASP}_hand_search
                      ${catkin_LIBRARIES})
                      
target_link_libraries(${GRASP}_point_list 
                      ${GRASP}_eigen_utils)

# Rename executable.
set_target_properties(${GRASP}_generate_candidates 
                      PROPERTIES OUTPUT_NAME generate_candidates 
                      PREFIX "")

set_target_properties(${GRASP}_grasp_candidates_generator 
                      PROPERTIES OUTPUT_NAME grasp_candidates_generator)

# add Grasp Gen Server
add_executable(grasp_gen_server src/grasp_gen_server.cpp)
target_link_libraries(grasp_gen_server 
                      ${catkin_LIBRARIES}
                      ${GRASP}_config_file 
                      ${GRASP}_hand_search 
                      ${GRASP}_grasp_candidates_generator)
add_dependencies(grasp_gen_server grasp_srv_gencpp)

# add Grasp Gen Client
add_executable(grasp_gen_client src/grasp_gen_client.cpp)
target_link_libraries(grasp_gen_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(grasp_gen_client grasp_srv_gencpp)

# add Render Object Node
add_executable(render_object src/render_object.cpp)
target_link_libraries(render_object ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# install
catkin_install_python(PROGRAMS scripts/grasp_test.py scripts/robot_kinematics_render.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
