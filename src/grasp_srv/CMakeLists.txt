cmake_minimum_required(VERSION 3.0.2)
project(grasp_srv)
set(CMAKE_CXX_STANDARD 14)
# Macro
set(GRASP "gpd")

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

#Opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Classifier depends on what libraries are installed.
option(USE_OPENVINO "use classifier based on OpenVINO toolkit" OFF)
option(USE_CAFFE "use classifier based on Caffe framework" OFF)
option(USE_OPENCV "use classifier based on OpenCV framework" OFF)
if(USE_OPENVINO STREQUAL "ON")
  find_package(InferenceEngine 1.0)
  if (NOT InferenceEngine_FOUND)
    message(FATAL_ERROR "Please install OpenVINO https://software.intel.com/en-us/articles/OpenVINO-Install-Linux")
  endif()
  include_directories(${InferenceEngine_INCLUDE_DIRS})
  link_directories(${InferenceEngine_LIBRARY_DIRS})
  add_definitions(-DUSE_OPENVINO)
  get_filename_component(MODELS_DIR "models/openvino" ABSOLUTE)
  configure_file(include/gpd/net/openvino_classifier.h.in gpd/net/openvino_classifier.h)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  set(classifier_src src/${GRASP}/net/classifier.cpp src/${GRASP}/net/openvino_classifier.cpp)
  set(classifier_dep ${InferenceEngine_LIBRARIES})
  message("Using OpenVINO")
elseif(USE_CAFFE STREQUAL "ON")
  find_package(Caffe)
  if (NOT Caffe_FOUND)
    message(FATAL_ERROR "Please install Caffe https://caffe.berkeleyvision.org/installation.html")
  endif()
  include_directories(${Caffe_INCLUDE_DIRS})
  add_definitions(${Caffe_DEFINITIONS})
  add_definitions(-DUSE_CAFFE)
  set(classifier_src src/${GRASP}/net/classifier.cpp src/${GRASP}/net/caffe_classifier.cpp)
  set(classifier_dep ${Caffe_LIBRARIES} ${OpenCV_LIBRARIES})
elseif(USE_OPENCV STREQUAL "ON")
  message(FATAL_ERROR "Not supported yet")
else()
  add_library(${GRASP}_conv_layer src/${GRASP}/net/conv_layer.cpp)
  add_library(${GRASP}_dense_layer src/${GRASP}/net/dense_layer.cpp)
  set(classifier_src src/${GRASP}/net/classifier.cpp src/${GRASP}/net/eigen_classifier.cpp)
  set(classifier_dep ${GRASP}_conv_layer ${GRASP}_dense_layer ${OpenCV_LIBRARIES})
endif()

# Optional PCL GPU operations
option(USE_PCL_GPU "use PCL GPU for point cloud operations" OFF)
if(USE_PCL_GPU STREQUAL "ON")
  FIND_PACKAGE(CUDA)
  if (NOT CUDA_FOUND)
    message(FATAL_ERROR "Please install CUDA and compile PCL with Nvidia GPU flags http://pointclouds.org/documentation/tutorials/gpu_install.php")
  endif()
  INCLUDE(FindCUDA)
  add_definitions(-DUSE_PCL_GPU)
  message("Using the GPU with PCL")
endif()

# Optional data generation (requires OpenCV Contrib)
option(BUILD_DATA_GENERATION "build data generation (requires OpenCV Contrib for HDF5)" OFF)
if(BUILD_DATA_GENERATION STREQUAL "ON")
  add_library(${GRASP}_data_generator src/${GRASP}/data_generator.cpp)
  target_link_libraries(${GRASP}_data_generator
   ${GRASP}_grasp_detector
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS})
  add_executable(${GRASP}_generate_data src/generate_data.cpp)
  target_link_libraries(${GRASP}_generate_data
   ${GRASP}_data_generator)
  message("Building data generation module")
endif()

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

# Generate the shared library from the sources
add_library(${GRASP}_grasp_detector SHARED src/${GRASP}/grasp_detector.cpp)

add_library(${GRASP}_classifier ${classifier_src})
target_link_libraries(${GRASP}_classifier
                      ${classifier_dep})

add_library(${GRASP}_clustering src/${GRASP}/clustering.cpp)
add_library(${GRASP}_sequential_importance_sampling src/${GRASP}/sequential_importance_sampling.cpp)

# namespace candidate
add_library(${GRASP}_antipodal src/${GRASP}/candidate/antipodal.cpp)
add_library(${GRASP}_candidates_generator src/${GRASP}/candidate/candidates_generator.cpp)
add_library(${GRASP}_finger_hand src/${GRASP}/candidate/finger_hand.cpp)
add_library(${GRASP}_frame_estimator src/${GRASP}/candidate/frame_estimator.cpp)
add_library(${GRASP}_hand src/${GRASP}/candidate/hand.cpp)
add_library(${GRASP}_hand_set src/${GRASP}/candidate/hand_set.cpp)
add_library(${GRASP}_hand_geometry src/${GRASP}/candidate/hand_geometry.cpp)
add_library(${GRASP}_hand_search src/${GRASP}/candidate/hand_search.cpp)
add_library(${GRASP}_local_frame src/${GRASP}/candidate/local_frame.cpp)

# namespace util
add_library(${GRASP}_cloud src/${GRASP}/util/cloud.cpp)
add_library(${GRASP}_config_file src/${GRASP}/util/config_file.cpp)
add_library(${GRASP}_eigen_utils src/${GRASP}/util/eigen_utils.cpp)
add_library(${GRASP}_plot src/${GRASP}/util/plot.cpp)
add_library(${GRASP}_point_list src/${GRASP}/util/point_list.cpp)

# namespace descriptor
add_library(${GRASP}_image_strategy STATIC src/${GRASP}/descriptor/image_strategy.cpp)
add_library(${GRASP}_image_1_channels_strategy STATIC src/${GRASP}/descriptor/image_1_channels_strategy.cpp)
add_library(${GRASP}_image_3_channels_strategy STATIC src/${GRASP}/descriptor/image_3_channels_strategy.cpp)
add_library(${GRASP}_image_12_channels_strategy STATIC src/${GRASP}/descriptor/image_12_channels_strategy.cpp)
add_library(${GRASP}_image_15_channels_strategy STATIC src/${GRASP}/descriptor/image_15_channels_strategy.cpp)
add_library(${GRASP}_image_geometry src/${GRASP}/descriptor/image_geometry.cpp)

add_library(${GRASP}_image_generator src/${GRASP}/descriptor/image_generator.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${GRASP}_clustering
${GRASP}_hand)

target_link_libraries(${GRASP}_grasp_detector
  ${GRASP}_clustering
  ${GRASP}_image_generator
  ${GRASP}_classifier
  ${GRASP}_candidates_generator
  ${GRASP}_hand_geometry
  ${GRASP}_hand_set
  ${GRASP}_config_file
  ${GRASP}_plot
  ${catkin_LIBRARIES})

# Linking for libraries
target_link_libraries(${GRASP}_antipodal
${GRASP}_point_list)

target_link_libraries(${GRASP}_cloud
  ${GRASP}_eigen_utils
  ${PCL_LIBRARIES})

target_link_libraries(${GRASP}_eigen_utils
${EIGEN_LIBRARIES})

target_link_libraries(${GRASP}_frame_estimator
  ${GRASP}_cloud
  ${GRASP}_local_frame)

target_link_libraries(${GRASP}_hand
${GRASP}_finger_hand)

target_link_libraries(${GRASP}_hand_set
  ${GRASP}_antipodal
  ${GRASP}_config_file
  ${GRASP}_hand
  ${GRASP}_hand_geometry
  ${GRASP}_local_frame
${GRASP}_point_list)

target_link_libraries(${GRASP}_hand_geometry
${GRASP}_config_file)

target_link_libraries(${GRASP}_hand_search
  ${GRASP}_antipodal
  ${GRASP}_cloud
  ${GRASP}_frame_estimator
  ${GRASP}_hand_set
  ${GRASP}_hand_geometry
  ${GRASP}_plot)

target_link_libraries(${GRASP}_plot
  ${GRASP}_cloud
  ${GRASP}_hand_set
  ${GRASP}_image_geometry
${GRASP}_local_frame)

target_link_libraries(${GRASP}_candidates_generator
  ${GRASP}_config_file
  ${GRASP}_hand_geometry
${GRASP}_hand_search)

target_link_libraries(${GRASP}_point_list
${GRASP}_eigen_utils)

target_link_libraries(${GRASP}_image_strategy
  ${GRASP}_image_geometry
  ${GRASP}_image_1_channels_strategy
  ${GRASP}_image_3_channels_strategy
  ${GRASP}_image_12_channels_strategy
  ${GRASP}_image_15_channels_strategy
  ${GRASP}_hand_set
${OpenCV_LIBS})


target_link_libraries(${GRASP}_image_1_channels_strategy
${GRASP}_image_strategy)

target_link_libraries(${GRASP}_image_3_channels_strategy
${GRASP}_image_strategy)

target_link_libraries(${GRASP}_image_12_channels_strategy
${GRASP}_image_strategy)

target_link_libraries(${GRASP}_image_15_channels_strategy
${GRASP}_image_strategy)

target_link_libraries(${GRASP}_image_geometry
${GRASP}_config_file)

target_link_libraries(${GRASP}_image_generator
  ${GRASP}_image_strategy
  ${GRASP}_hand_set
  ${GRASP}_image_strategy
  ${GRASP}_cloud
${GRASP}_eigen_utils)

target_link_libraries(${GRASP}_sequential_importance_sampling
  ${GRASP}_grasp_detector)

target_link_libraries(${GRASP}_image_generator
  ${GRASP}_hand_set
  ${GRASP}_image_strategy)


# add Grasp Gen Server
add_executable(grasp_gen_server src/grasp_gen_server.cpp)
target_link_libraries(grasp_gen_server 
                      ${catkin_LIBRARIES}
                      ${GRASP}_grasp_detector)
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
