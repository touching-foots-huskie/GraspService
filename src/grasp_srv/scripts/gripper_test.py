#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import math

if __name__ == "__main__":
    rospy.init_node('grasp_state_publisher')
    rate = rospy.Rate(10)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "tool0"
    t.transform.translation.x = 1.0
    t.transform.translation.y = 1.0
    t.transform.translation.z = 1.0

    x = 1.0
    y = 0.5
    z = 0.6
    w = 0.7
    _norm = math.sqrt(x**2 + y**2 + z**2 + w**2)
    x /= _norm
    y /= _norm
    z /= _norm
    w /= _norm
    t.transform.rotation.x = x
    t.transform.rotation.y = y
    t.transform.rotation.z = z
    t.transform.rotation.w = w

    for i in range(10):
        br.sendTransform(t)
        rate.sleep()