# Test the generation of grasp pose visually
#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import grasp_srv.srv
import grasp_srv.msg
import geometry_msgs.msg

import ikpy
from scipy.spatial.transform import Rotation as R

import robot_kinematics_render

'''
Pose is a 1D list of 7 elements (x, y, z, qx, qy, qz, qw), 
'''
def test_grasp_gen(model_name_list, object_pose_list):
    rospy.wait_for_service('grasp_gen')
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', grasp_srv.srv.GraspGen)
        object_poses = grasp_srv.msg.ObjectPoses()
        for name, pose in zip(model_name_list, object_pose_list):
            object_poses.object_names.append(name)
            object_pose = geometry_msgs.msg.Pose()
            # set position
            object_pose.position.x = pose[0]
            object_pose.position.y = pose[1]
            object_pose.position.z = pose[2]
            object_pose.orientation.x = pose[3]
            object_pose.orientation.y = pose[4]
            object_pose.orientation.z = pose[5]
            object_pose.orientation.w = pose[6]
            object_poses.object_poses.append(object_pose)

        resp = grasp_gen(object_poses)
        if len(ur5e.grasps.global_grasp_poses[0].grasp_poses) > 0:
            print("Service Sent")
            # Test on One
            grasp_pose = resp.grasps.global_grasp_poses[0].grasp_poses[0]
            r = R.from_quat([grasp_pose.orientation.x, 
                             grasp_pose.orientation.y,
                             grasp_pose.orientation.z,
                             grasp_pose.orientation.w])
            
            # run the render function
            render_robot_pose(r.as_matrix())
            print("Pose Rendered")
            return
        else:
            print("No Grasp Candidates Found")

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    model_name_list = ["a_cup"]
    origin_pos = [0.2, 0.2, 0.2, 0., 0., 0., 1.]
    object_pose_list = [origin_pos]
    test_grasp_gen(model_name_list, object_pose_list)

