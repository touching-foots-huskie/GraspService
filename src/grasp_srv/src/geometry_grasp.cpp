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
        size_r = (size_x + size_y) / 2.0;  // average radius
        size_h = size_z;
        center_h = (z_max + z_min) / 2.0;
        axis = z_axis;
        return;
    }

    if((gap_yz < gap_xy) && (gap_yz < gap_xz)) {
        size_r = (size_y + size_z) / 2.0;  // average radius
        size_h = size_x;
        center_h = (x_max + x_min) / 2.0;
        axis = x_axis;
        return;
    }

    if((gap_xz < gap_xy) && (gap_xz < gap_yz)) {
        size_r = (size_x + size_z) / 2.0;
        size_h = size_y;
        center_h = (y_max + y_min) / 2.0;
        axis = y_axis;
        return;
    }
}


// generate grasp pose
void can_grasp(MatrixArray& frame_array, VectorArray& position_array, std::string filename, 
               double scale, std::vector<bool>& block_list, double finger_len, double finger_gap,
               int num_angle, double slice_step, bool bb_pruning) {
    // parse information
    double size_r, size_h, center_h;
    AXIS axis;
    can_parse(filename, size_r, size_h, center_h, axis);
    // rescale size
    size_r *= scale;
    size_h *= scale;
    center_h *= scale;

    int num_slice_h = std::max(int(size_h / (2.0*slice_step)), 1);
    // retreat
    double retreat_r = -size_r + finger_len;
    double retreat_h = -size_h + finger_len;
    Eigen::Vector3d retreat_vector(1., 0., 0.);

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
        double angle = double(i) / double(num_angle) * 2.0 * M_PI;
        rot_z = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

        // horizontal
        Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d rot_position;
        
        position(0) =  -size_r / 2.0;
        // add slices
        double signs[2] = {1.0f, -1.0f};

        // local rot
        MatrixArray rot_xs;
        Eigen::Matrix3d id_x = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d rot_x;
        rot_x = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
        rot_xs.push_back(id_x);
        rot_xs.push_back(rot_x);
        // deciding if block horizontal pose
        
        for(int w = 0; w < 2; ++w) {
            if(block_list[w]) continue;  // block certain direction
            Eigen::Matrix3d local_rot = rot_xs[w];
            for(int j = 0; j < num_slice_h; ++j) {
                for(int k = 0; k < 2; ++k) {
                    frame = w_rot * rot_z * local_rot;

                    // bb_checking
                    if(bb_pruning) {
                        Eigen::Vector3d edge_vector(0.0, 1.0, 0.0);
                        edge_vector = frame * edge_vector;
                        Eigen::Vector3d size_vector(size_r, size_r, size_h);
                        size_vector = w_rot * size_vector;
                        edge_vector = edge_vector.cwiseProduct(size_vector);
                        if(edge_vector.norm() >= finger_gap) break;
                    }

                    frame_array.push_back(frame);
                    position(2) = signs[k] * double(j) * slice_step + center_h;
                    rot_position = frame * position;
                    // retreat
                    double retreat_len;
                    if(size_r > finger_len) {
                        retreat_len = 0.;
                    }
                    else {
                        retreat_len = retreat_r;
                    }
                    rot_position -= frame * retreat_len * retreat_vector;
                    position_array.push_back(rot_position);
                    if(j == 0 || w == 1) break;
                }
                if(w == 1) break;
            }
        }
        
        

        // vertical
        position = Eigen::Vector3d::Zero();;
        for(int k = 0; k < 2; ++k) {
            if(block_list[k+2]) continue;
            rot_y = Eigen::AngleAxisd(signs[k] * M_PI/2.0, Eigen::Vector3d::UnitY());
            frame = w_rot * rot_z * rot_y;
            // retreat
            if(size_h > finger_len) 
                position(0) =  -size_h/2.0 - center_h;
            else
                position(0) = -size_h/2.0 - center_h - retreat_h;
            rot_position = frame * position;

            // bb_checking
            if(bb_pruning) {
                Eigen::Vector3d edge_vector(0.0, 1.0, 0.0);
                edge_vector = frame * edge_vector;
                Eigen::Vector3d size_vector(size_r, size_r, size_h);
                size_vector = w_rot * size_vector;
                edge_vector = edge_vector.cwiseProduct(size_vector);
                if(edge_vector.norm() >= finger_gap) break;
            }

            frame_array.push_back(frame);
            position_array.push_back(rot_position);
        }
    }
    return;
}


// ReIm: generate grasp pose
void box_grasp(MatrixArray& frame_array, VectorArray& position_array, std::string filename, 
               double scale, std::vector<bool>& block_list, double finger_len, double finger_gap,
               double slice_step, bool bb_pruning) {
    // parse information
    double size_x, size_y, size_z;
    double center_x, center_y, center_z;
    box_parse(filename, size_x, size_y, size_z, center_x, center_y, center_z);
    
    // rescale size & centers
    size_x *= scale;
    size_y *= scale;
    size_z *= scale;
    center_x *= scale;
    center_y *= scale;
    center_z *= scale;
    double sizes[3] = {size_x, size_y, size_z};
    double centers[3] = {center_x, center_y, center_z};

    // slices
    int num_slice_x = std::max(int(size_x /(2.0 * slice_step)), 1);
    int num_slice_y = std::max(int(size_y /(2.0 * slice_step)), 1);
    int num_slice_z = std::max(int(size_z /(2.0 * slice_step)), 1);
    Eigen::Vector3i num_slices(num_slice_x, num_slice_y, num_slice_z);

    // retreat distance
    double retreat_x = -size_x + finger_len;
    double retreat_y = -size_y + finger_len;
    double retreat_z = -size_z + finger_len;
    double retreats[3] = {retreat_x, retreat_y, retreat_z};

    // signs
    double signs[2] = {-1., 1.};

    // frames & poses
    MatrixArray frames;
    VectorArray poses;
    frames.reserve(6);
    Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rot;
    // +x
    frames[0] = frame;
    // -x
    frame = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    frames[1] = frame;
    // +y
    frame = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());
    frames[2] = frame;
    // -y
    frame = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ());
    frames[3] = frame;
    // +z
    frame = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());
    frames[4] = frame;
    // -z
    frame = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
    frames[5] = frame;

    // poses
    for(int di = 0; di < 3; ++di) {
        for(int si = 0; si < 2; ++si) {
            Eigen::Vector3d position;
            // position
            position << center_x, center_y, center_z;  // Assign center
            position[di] = signs[si] * sizes[di] / 2.0 + centers[di];
            // direction
            poses.push_back(position);
        }
    }

    // slice & rot
    MatrixArray rot_xs;
    Eigen::Matrix3d id_x = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rot_x;
    rot_x = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
    rot_xs.push_back(id_x);
    rot_xs.push_back(rot_x);

    Eigen::Vector3d shape_size;
    shape_size << size_x, size_y, size_z;

    for(int i = 0; i < 6; ++i) {
        Eigen::Matrix3d frame_i = frames[i];
        Eigen::Vector3d pose_i = poses[i];
        for(int j = 0; j < 2; ++j) {
            // block grasp pose
            if(block_list[2 * i + j]) continue;

            Eigen::Matrix3d local_rot = rot_xs[j];
            Eigen::Matrix3d frame_ij = frame_i * local_rot;
            Eigen::Vector3d pose_ij;
            // slices
            Eigen::Vector3d slice_vector(0., 0., 1.);
            slice_vector = frame_ij * slice_vector;
            Eigen::Vector3i num_slice_vector = slice_vector.cast<int>();
            slice_vector = slice_vector * slice_step; 
            num_slice_vector = num_slice_vector.cwiseProduct(num_slices);
            for(int k = 0; k < num_slice_vector.cwiseAbs().maxCoeff(); ++k) {
                for(int s = 0; s < 2; ++s) {
                    pose_ij = pose_i + signs[s] * double(k) * slice_vector;
                    // retreat
                    Eigen::Vector3d retreat_vector(1.0, 0.0, 0.0);
                    retreat_vector = frame_ij * retreat_vector;
                    // find retreat direction
                    for(int r = 0; r < 3; ++r) {
                        if(fabs(retreat_vector(r)) > 0.0001) {
                            double retreat_len = 0.;
                            if(finger_len < sizes[r]) {
                                // no retreat
                                retreat_len = 0.;
                            }
                            else {
                                retreat_len = retreats[r];
                            }
                            pose_ij = pose_ij - retreat_vector * retreat_len;
                        }
                    }

                    // bb_checking
                    if(bb_pruning) {
                        Eigen::Vector3d edge_vector(0.0, 1.0, 0.0);
                        edge_vector = frame_ij * edge_vector;
                        Eigen::Vector3d size_vector(sizes[0], sizes[1], sizes[2]);
                        edge_vector = edge_vector.cwiseProduct(size_vector);
                        double edge_len = edge_vector.norm();
                        if(edge_len >= finger_gap) break;
                    }

                    position_array.push_back(pose_ij);
                    frame_array.push_back(frame_ij);
                    if(k == 0) break;  // not repeating k==0
                }
            }
        }
    }
}


// generate grasp pose
void bowl_grasp(MatrixArray& frame_array, VectorArray& position_array, 
                std::string filename, double scale, double finger_len, double finger_gap, int num_angle) {
    // parse information
    double size_r, size_h, center_h;
    AXIS axis;
    can_parse(filename, size_r, size_h, center_h, axis);
    // rescale size
    size_r *= scale;
    size_h *= scale;
    center_h *= scale;
    double size_d = std::sqrt(size_h*size_h + size_r*size_r/4.0);
    double bottom = center_h - (size_h/2.0);
    Eigen::Vector3d bottom_vector;
    bottom_vector << 0., 0., bottom;

    double angle_y = std::atan2(size_r/2., size_h);
    double size_retreat = size_d - std::tan(angle_y) * finger_gap / 2.;
    double retreat_d = -size_retreat + finger_len;
    Eigen::Vector3d retreat_vector(1., 0., 0.);

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
        Eigen::Matrix3d rot_z, rot_y, rot_x;
        double angle = double(i) / double(num_angle) * 2.0 * M_PI;
        rot_z = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());

        // vertical
        rot_y = Eigen::AngleAxisd(M_PI/2.0-angle_y, Eigen::Vector3d::UnitY());
        rot_x = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d frame = w_rot * rot_z * rot_y * rot_x;
        // retreat
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        if(size_retreat > finger_len) 
            position(0) =  - size_d;
        else
            position(0) = - size_d - retreat_d;
        Eigen::Vector3d rot_position = frame * position;
        rot_position += w_rot * bottom_vector;
        frame_array.push_back(frame);
        position_array.push_back(rot_position);
        
    }
    return;
}


// generate grasp pose
void square_bowl_grasp(MatrixArray& frame_array, VectorArray& position_array, 
                       std::string filename, double scale, std::vector<bool>& block_list, double finger_len, double finger_gap, int num_angle) {
    // parse information
    double size_x, size_y, size_z;
    double center_x, center_y, center_z;
    box_parse(filename, size_x, size_y, size_z, center_x, center_y, center_z);
    
    // rescale size & centers
    size_x *= scale;
    size_y *= scale;
    size_z *= scale;
    center_x *= scale;
    center_y *= scale;
    center_z *= scale;

    // frames & poses
    MatrixArray frames;
    VectorArray poses;

    Eigen::Vector3d position;
    position << center_x, center_y + size_y/2., center_z + size_z/2.;
    poses.push_back(position);
    position << center_x + size_x/2., center_y, center_z + size_z/2.;
    poses.push_back(position);
    position << center_x, center_y - size_y/2., center_z + size_z/2.;
    poses.push_back(position);
    position << center_x - size_x/2., center_y, center_z + size_z/2.;
    poses.push_back(position);
    
    // local rot
    MatrixArray rot_xs;
    Eigen::Matrix3d id_x = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rot_x;
    rot_x = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
    rot_xs.push_back(id_x);
    rot_xs.push_back(rot_x);

    // frame
    Eigen::Matrix3d frame;
    frame = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());

    double retreat_len;
    for(int i = 0; i < 2; ++i) {    
        for(int j = 0; j < 2; ++j) {
            // block grasp pose
            if(block_list[2*i+j]) continue;
            Eigen::Matrix3d local_rot = rot_xs[j];
            Eigen::Matrix3d frame_ij = frame * local_rot;
            Eigen::Vector3d pose_ij = poses[2*i+j];
            if(finger_len < size_z/2.) {
                // no retreat
                retreat_len = 0.;
            }
            else {
                retreat_len = -size_z/2. + finger_len;
            }
            pose_ij(2) += retreat_len;
            position_array.push_back(pose_ij);
            frame_array.push_back(frame_ij);
        }
    }
    return;
}