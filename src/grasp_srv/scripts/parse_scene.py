'''
Load scene, send infomation to object_render
Seg pointcloud
'''
#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import open3d as o3d
import json


if __name__=='__main__':
    scene_name = "1-4"
    data_path = "/home/harvey/Data/kinect_data/output/"
    image_path = data_path + scene_name
    depth_img_path = image_path + "/depth.png"
    color_img_path = image_path + "/rgb.png"
    seg_img_path   = image_path + "/seg.png"

    # load object_id 
    with open(data_path + "object_id.json", "r") as f:
        object_id = json.load(f)

    # load object poses
    with open(data_path + "info/" + scene_name + ".json", "r") as f:
        object_infos = json.load(f)

    # load segmeneted pointcloud
    depth = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)
    color = cv2.imread(color_img_path, cv2.IMREAD_COLOR)
    seg   = cv2.imread(seg_img_path, cv2.IMREAD_UNCHANGED)
    seg   = seg[:, :, 0]  # only one channel is useful 

    labels = np.unique(seg)

    # patch pose_world, kinect_pose, model_name together
    object_datas = list()
    for i, label in enumerate(labels):
        if label != 0:
            name = object_id[str(label)]
            for object_info in object_infos:
                if object_info['name'] == name:
                    object_data = object_info
                    object_datas.append(object_data)
                    break

    with open(image_path + "/object_data.json", "w") as f:
        json.dump(object_datas, f)

    # do pointcloud segmentation 
    for i, label in enumerate(labels):
        # get mask
        if label != 0:
            mask = (seg == label)
            masked_depth = depth.copy()
            masked_depth[~mask] = 0
        else:
            masked_depth = depth.copy()
        
        # filt the depth image
        kernel_size = 11
        kernel = np.ones((kernel_size, kernel_size), np.float32) / kernel_size**2
        masked_depth.astype(np.float);
        masked_depth = masked_depth / 10000.0
        # masked_depth = cv2.filter2D(masked_depth, -1, kernel)
        masked_depth = (10000 *  masked_depth).astype(np.uint16)

        cam_intrinsics = np.array([
            [1120.1199, 0, 640.5],
            [0, 1120.1199, 360.5],
            [0, 0, 1]
        ])
        color_o3d = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(masked_depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=10000, convert_rgb_to_intensity=False)
        intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic()
        intrinsics_o3d.intrinsic_matrix = cam_intrinsics
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics_o3d)
        if label != 0:
            output_file_name = image_path + "/{}.pcd".format(i-1)
        else:
            output_file_name = image_path + "/bg.pcd"

        if not pcd.is_empty():
            print("{} Writed".format(output_file_name))
            o3d.io.write_point_cloud(output_file_name, pcd)
            # o3d.visualization.draw_geometries([pcd])

