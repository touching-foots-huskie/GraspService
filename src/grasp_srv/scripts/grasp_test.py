# Test the generation of grasp pose visually
#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from grasp_srv.srv import *
from grasp_srv.msg import *
from geometry_msgs.msg import *


'''
Pose is a 1D list of 7 elements (x, y, z, qx, qy, qz, qw), 
'''
def test_grasp_gen(model_name_list, object_pose_list):
    rospy.wait_for_service('grasp_gen')
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', GraspGen)
        ObjectPoses object_poses()
        for name, pose in zip(model_name_list, object_pose_list):
            object_poses.object_names.append(name)
            Pose object_pose()
            # set position
            object_pose.poistion    = [pose[0], pose[1], pose[2]]
            object_pose.orientation = [pose[3], pose[4], pose[5], pose[6]]
            object_poses.object_poses.append(object_pose)
        resp = grasp_gen(object_poses)
        return

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    model_name_list = ["a_cup"]
    origin_pos = [0., 0., 0., 0., 0., 0., 1.]
    object_pose_list = [origin_pos]
    test_grasp_gen(model_name_list, object_pose_list)

