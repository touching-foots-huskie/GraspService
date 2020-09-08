#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String

def publish_gripper(quat, trans, gripper_id):
    # set initialization
    rate = rospy.Rate(10)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "tool0"
    t.transform.translation.x = trans.x
    t.transform.translation.y = trans.y
    t.transform.translation.z = trans.z

    t.transform.rotation.x = quat.x
    t.transform.rotation.y = quat.y
    t.transform.rotation.z = quat.z
    t.transform.rotation.w = quat.w

    for i in range(10):
        br.sendTransform(t)
        rate.sleep()


def publish_object(object_pose, object_name):
    # set initialization
    rate = rospy.Rate(10)
    obj_pub = rospy.Publisher("object_name", String, queue_size=10)
    pose_pub = rospy.Publisher("object_pose", Pose, queue_size=10)

    # prepare data
    object_name_msg = String()
    object_name_msg.data = object_name

    for i in range(10):
        obj_pub.publish(object_name_msg)
        pose_pub.publish(object_pose)
        rate.sleep()


if __name__ == '__main__':
    pass