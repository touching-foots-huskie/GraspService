#include "geometry_grasp.hpp"


// geometry parse
void box_parse(std::string filename, double& size_x, double& size_y, double& size_z) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't find model data \n");
        return;
    }

    // check lowest and highest in z-axis
    double bound = 100.0;
    double x_min, x_max, y_min, y_max, z_min, z_max; 
    x_max = y_max = z_max = -bound;
    x_min = y_min = z_min =  bound;
    for(auto p : cloud->points) {
        // min
        x_min = (p.x >= x_min) ? x_min : p.x; 
        y_min = (p.y >= y_min) ? y_min : p.y; 
        z_min = (p.z >= z_min) ? z_min : p.z;
        // max
        x_max = (p.x <= x_max) ? x_max : p.x;
        y_max = (p.y <= y_max) ? y_max : p.y;
        z_max = (p.z <= z_max) ? z_max : p.z;
    }
    size_x = x_max - x_min;
    size_y = y_max - y_min;
    size_z = z_max - z_min;
    return;
}


void can_parse(std::string filename, double& size_r, double& size_h, AXIS& axis) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't find model data \n");
        return;
    }

    // check lowest and highest in z-axis
    double bound = 100.0;
    double x_min, x_max, y_min, y_max, z_min, z_max; 
    x_max = y_max = z_max = -bound;
    x_min = y_min = z_min =  bound;
    for(auto p : cloud->points) {
        // min
        x_min = (p.x >= x_min) ? x_min : p.x; 
        y_min = (p.y >= y_min) ? y_min : p.y; 
        z_min = (p.z >= z_min) ? z_min : p.z;
        // max
        x_max = (p.x <= x_max) ? x_max : p.x;
        y_max = (p.y <= y_max) ? y_max : p.y;
        z_max = (p.z <= z_max) ? z_max : p.z;
    }

    double size_x, size_y, size_z;
    size_x = x_max - x_min;
    size_y = y_max - y_min;
    size_z = z_max - z_min;

    double gap_xy, gap_yz, gap_xz;
    gap_xy = std::fabs(size_x - size_y);
    gap_yz = std::fabs(size_y - size_z);
    gap_xz = std::fabs(size_x - size_z);

    if((gap_xy < gap_yz) && (gap_xy < gap_xz)) {
        size_r = size_x;
        size_h = size_z;
        axis = z_axis;
        return;
    }

    if((gap_yz < gap_xy) && (gap_yz < gap_xz)) {
        size_r = size_y;
        size_h = size_x;
        axis = x_axis;
        return;
    }

    if((gap_xz < gap_xy) && (gap_xz < gap_yz)) {
        size_r = size_x;
        size_h = size_y;
        axis = y_axis;
        return;
    }
}


// generate grasp pose
void box_grasp(MatrixArray& frame_array, VectorArray& position_array,
               std::string filename, double scale) {
    // parse information
    double size_x, size_y, size_z;
    box_parse(filename, size_x, size_y, size_z);
    // rescale size
    size_x *= scale;
    size_y *= scale;
    size_z *= scale;

    // grasp along +x-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        position(0) =  -size_x / 2.0;
        frame_array.push_back(frame);
        position_array.push_back(position);
    }
    // grasp along -x-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        frame = frame * rot_z;
        position(0) =  size_x / 2.0;
        frame_array.push_back(frame);
        position_array.push_back(position);
    }

    // grasp along +y-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());
        frame = frame * rot_z;
        position(1) =  -size_y / 2.0;
        frame_array.push_back(frame);
        position_array.push_back(position);
    }
    // grasp along -y-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ());
        frame = frame * rot_z;
        position(1) =  size_y / 2.0;
        frame_array.push_back(frame);
        position_array.push_back(position);
    }

    // grasp along +z-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot_y;
        rot_y = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());
        frame = frame * rot_y;
        position(2) =  -size_z / 2.0;
        frame_array.push_back(frame);
        position_array.push_back(position);
    }
    // grasp along -z-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot_y;
        rot_y = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
        frame = frame * rot_y;
        position(2) =  size_z / 2.0;
        frame_array.push_back(frame);
        position_array.push_back(position);
    }
    return;
}


// generate grasp pose
void can_grasp(MatrixArray& frame_array, VectorArray& position_array,
               std::string filename, double scale, int num_angle) {
    // parse information
    double size_r, size_h;
    AXIS axis;
    can_parse(filename, size_r, size_h, axis);
    // rescale size
    size_r *= scale;
    size_h *= scale;
    
    // choose axis
    Eigen::Matrix3d w_rot = Eigen::Matrix3d::Identity();
    switch (axis)
    {
    case x_axis:
        w_rot = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());
        break;
    case y_axis:
        w_rot = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
        break;
    case z_axis:
        break;
    default:
        break;
    }
    // grasp along +r-axis 
    for(int i = 0; i < num_angle; ++i)
    {
        Eigen::Matrix3d rot_z, rot_y;
        float angle = float(i) / float(num_angle) * M_PI;
        rot_z = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

        // horizontal
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        
        position(0) =  -size_r / 2.0;
        frame = w_rot * rot_z * frame;
        position = w_rot * rot_z * position;
        frame_array.push_back(frame);
        position_array.push_back(position);

        // vertical 1
        Eigen::Matrix3d frame_v1 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position_v1 = Eigen::Vector3d::Zero();
        position_v1(2) =  size_h / 2.0;
        rot_y = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
        frame_v1 = w_rot * frame_v1 * rot_y;
        frame_v1 = rot_z * frame_v1;
        position_v1 = w_rot * rot_z * position_v1;
        frame_array.push_back(frame_v1);
        position_array.push_back(position_v1);

        // vertical 2
        Eigen::Matrix3d frame_v2 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position_v2 = Eigen::Vector3d::Zero();
        position_v2(2) =  -size_h / 2.0;
        rot_y = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());
        frame_v2 = w_rot * frame_v2 * rot_y;
        frame_v2 = rot_z * frame_v2;
        position_v2 = w_rot * rot_z * position_v2;
        frame_array.push_back(frame_v2);
        position_array.push_back(position_v2);
    }
    return;
}