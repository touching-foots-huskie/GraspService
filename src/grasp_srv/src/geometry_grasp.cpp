#include "geometry_grasp.hpp"


// geometry parse
void box_parse(std::string filename, double& size_x, double& size_y, double& size_z,
               double& center_x, double& center_y, double& center_z) {
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
    center_x = (x_max + x_min) / 2.0;
    center_y = (y_max + y_min) / 2.0;
    center_z = (z_max + z_min) / 2.0;
    return;
}


void can_parse(std::string filename, double& size_r, double& size_h, 
               double& center_h, AXIS& axis) {
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
        center_h = (z_max + z_min) / 2.0;
        axis = z_axis;
        return;
    }

    if((gap_yz < gap_xy) && (gap_yz < gap_xz)) {
        size_r = size_y;
        size_h = size_x;
        center_h = (x_max + x_min) / 2.0;
        axis = x_axis;
        return;
    }

    if((gap_xz < gap_xy) && (gap_xz < gap_yz)) {
        size_r = size_x;
        size_h = size_y;
        center_h = (y_max + y_min) / 2.0;
        axis = y_axis;
        return;
    }
}


// generate grasp pose
void box_grasp(MatrixArray& frame_array, VectorArray& position_array,
               std::string filename, double scale, int num_slice) {
    // parse information
    double size_x, size_y, size_z;
    double center_x, center_y, center_z;
    box_parse(filename, size_x, size_y, size_z, center_x, center_y, center_z);
    // rescale size
    size_x *= scale;
    size_y *= scale;
    size_z *= scale;
    center_x *= scale;
    center_y *= scale;
    center_z *= scale;

    // slices
    double slice_x = size_x / (2.0*(double)num_slice);
    double slice_y = size_y / (2.0*(double)num_slice);
    double slice_z = size_z / (2.0*(double)num_slice);
    // rot_x
    Eigen::Matrix3d rot_x;
    rot_x = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());

    // grasp along +x-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position;
        position << center_x, center_y, center_z;
        position(0) = -size_x / 2.0 + center_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(2) = double(i) * slice_z + center_z;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(2) = -double(i) * slice_z + center_z;
            position_array.push_back(position);
        }
        position(2) = center_z;

        // vertical grasp pose
        frame = frame * rot_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(1) = double(i) * slice_y + center_y;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(1) = -double(i) * slice_y + center_y;
            position_array.push_back(position);
        }
        position(1) = center_y;
    }
    // grasp along -x-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position;
        position << center_x, center_y, center_z;
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        frame = frame * rot_z;
        position(0) =  size_x / 2.0 + center_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(2) = double(i) * slice_z + center_z;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(2) = -double(i) * slice_z + center_z;
            position_array.push_back(position);
        }
        position(2) = center_z;

        // vertical grasp pose
        frame = frame * rot_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(1) = double(i) * slice_y + center_y;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(1) = -double(i) * slice_y + center_y;
            position_array.push_back(position);
        }
        position(1) = center_y;
    }

    // grasp along +y-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position;
        position << center_x, center_y, center_z;
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());
        frame = frame * rot_z;
        position(1) =  -size_y / 2.0 + center_y;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(2) = double(i) * slice_z + center_z;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(2) = -double(i) * slice_z + center_z;
            position_array.push_back(position);
        }
        position(2) = center_z;

        // vertical grasp pose
        frame = frame * rot_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(0) = double(i) * slice_x + center_x;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(0) = -double(i) * slice_x + center_x;
            position_array.push_back(position);
        }
        position(0) = center_x;
    }
    // grasp along -y-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position;
        position << center_x, center_y, center_z;
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ());
        frame = frame * rot_z;
        position(1) =  size_y / 2.0 + center_y;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(2) = double(i) * slice_z + center_z;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(2) = -double(i) * slice_z + center_z;
            position_array.push_back(position);
        }
        position(2) = center_z;

        // vertical grasp pose
        frame = frame * rot_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(0) = double(i) * slice_x + center_x;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(0) = -double(i) * slice_x + center_x;
            position_array.push_back(position);
        }
        position(0) = center_x;
    }

    // grasp along +z-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position;
        position << center_x, center_y, center_z;
        Eigen::Matrix3d rot_y;
        rot_y = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());
        frame = frame * rot_y;
        position(2) =  -size_z / 2.0 + center_z;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(0) = double(i) * slice_x + center_x;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(0) = -double(i) * slice_x + center_x;
            position_array.push_back(position);
        }
        position(0) = center_x;

        // vertical grasp pose
        frame = frame * rot_x;
        frame_array.push_back(frame);
        position_array.push_back(position);
        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(1) = double(i) * slice_y + center_y;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(1) = -double(i) * slice_y + center_y;
            position_array.push_back(position);
        }
        position(1) = center_y;
    }
    // grasp along -z-axis 
    {
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position;
        position << center_x, center_y, center_z;
        Eigen::Matrix3d rot_y;
        rot_y = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
        frame = frame * rot_y;
        position(2) =  size_z / 2.0 + center_z;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(0) = double(i) * slice_x + center_x;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(0) = -double(i) * slice_x + center_x;
            position_array.push_back(position);
        }
        position(0) = center_x;

        // vertical grasp pose
        frame = frame * rot_x;
        frame_array.push_back(frame);
        position_array.push_back(position);

        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(1) = double(i) * slice_y + center_y;
            position_array.push_back(position);
            // -
            frame_array.push_back(frame);
            position(1) = -double(i) * slice_y + center_y;
            position_array.push_back(position);
        }
        position(1) = center_y;
    }
    return;
}


// generate grasp pose
void can_grasp(MatrixArray& frame_array, VectorArray& position_array,
               std::string filename, double scale, int num_angle, int num_slice) {
    // parse information
    double size_r, size_h, center_h;
    AXIS axis;
    can_parse(filename, size_r, size_h, center_h, axis);
    // rescale size
    size_r *= scale;
    size_h *= scale;
    center_h *= scale;

    double slice_h = size_h / (2.0 * double(num_slice));
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
        float angle = float(i) / float(num_angle) * 2.0 * M_PI;
        rot_z = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

        // horizontal
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d rot_position;
        position(2) = center_h;
        
        position(0) =  -size_r / 2.0;
        frame = w_rot * rot_z * frame;
        rot_position = w_rot * rot_z * position;
        frame_array.push_back(frame);
        position_array.push_back(rot_position);

        // add slices
        for(int i = 1; i < num_slice; ++i) {
            // +
            frame_array.push_back(frame);
            position(2) = double(i) * slice_h + center_h;
            rot_position = w_rot * rot_z * position;
            position_array.push_back(rot_position);
            // -
            frame_array.push_back(frame);
            position(2) = -double(i) * slice_h + center_h;
            rot_position = w_rot * rot_z * position;
            position_array.push_back(rot_position);
        }

        // vertical 1
        Eigen::Matrix3d frame_v1 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position_v1 = Eigen::Vector3d::Zero();
        position_v1(2) =  size_h / 2.0 + center_h;
        rot_y = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
        frame_v1 = w_rot * rot_z * rot_y;
        position_v1 = w_rot * rot_z * position_v1;
        frame_array.push_back(frame_v1);
        position_array.push_back(position_v1);

        // vertical 2
        Eigen::Matrix3d frame_v2 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position_v2 = Eigen::Vector3d::Zero();
        position_v2(2) =  -size_h / 2.0 + center_h;
        rot_y = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());
        frame_v2 = w_rot * rot_z * rot_y;
        position_v2 = w_rot * rot_z * position_v2;
        frame_array.push_back(frame_v2);
        position_array.push_back(position_v2);
    }
    return;
}