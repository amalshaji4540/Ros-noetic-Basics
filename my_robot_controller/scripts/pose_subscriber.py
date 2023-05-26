#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pos_callback(msg : Pose):
    rospy.loginfo("("+str(msg.x)+ "," +str(msg.y)+")")


if __name__=='__main__':
    rospy.init_node("turtle_pose_subscriber")
    sub = rospy.Subscriber("/turtle1/pose",Pose,callback=pos_callback)

    rospy.loginfo("turtle_pose_subscriber has started")
    rospy.spin()