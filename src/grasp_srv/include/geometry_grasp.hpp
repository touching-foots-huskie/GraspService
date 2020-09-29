#ifndef GEOMETRY_GRASP_HPP
#define GEOMETRY_GRASP_HPP
// eigen
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>

using VectorArray = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using MatrixArray = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;

enum AXIS{x_axis, y_axis, z_axis};

/*
TODO: Add Bowl, Plate
*/

void box_parse(std::string filename, double& size_x, double& size_y, double& size_z,
               double& center_x, double& center_y, double& center_z);
void can_parse(std::string filename, double& size_r, double& size_h, 
               double& center_h, AXIS& axis);

// generate grasp pose by geometry
void box_grasp(MatrixArray& frame_array, VectorArray& position_array,
               std::string filename, double scale, double finger_len=0.035, double slice_thresh=0.05, int num_slice=2);
void can_grasp(MatrixArray& frame_array, VectorArray& position_array,
               std::string filename, double scale, double finger_len=0.035, double slice_thresh=0.05, int num_angle=8, int num_slice=2);

#endif