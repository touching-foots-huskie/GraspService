'''
Load scene, send infomation to object_render
Seg pointcloud
'''

#!/usr/bin/env python3
import sys
import rospy
import cv2
import numpy as np
import open3d as o3d


if __name__=='__main__':
    image_path = "/home/harvey/Data/kinect_data/output/1-1/"
    depth_img_path = image_path + "depth.png"
    color_img_path = image_path + "rgb.png"
    seg_img_path   = image_path + "seg.png"
    depth = cv2.imread(depth_img_path, cv2.IMREAD_COLOR)
    color = cv2.imread(color_img_path, cv2.IMREAD_UNCHANGED)
    seg   = cv2.imread(seg_img_path, cv2.IMREAD_UNCHANGED)

    labels = np.unique(seg)
    for label in labels:
        if label != 0:
            mask = (seg == label)
            masked_depth = depth
            masked_depth[mask] = 0

            # depth = cv2.fastNlMeansDenoising(depth, None, 10, 7, 21)
            kernel_size = 11
            kernel = np.ones((kernel_size, kernel_size), np.float32) / kernel_size**2
            masked_depth = cv2.filter2D(depth, -1, masked_depth)
            masked_depth = (10000 * masked_depth).astype(np.uint16)

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
    
