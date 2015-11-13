#!/usr/bin/env python

import rospy
from tf import TransformListener
import geometry_msgs.msg


class TransformS(object):
    
    def __init__(self):
        rospy.init_node('stick_transform')
        self.tf = TransformListener()
        print "Starting tranformer"
        rospy.Subscriber("/whycon/poses",geometry_msgs.msg.PoseArray, self.handle_whycon)
        self.stick_pose_pub = rospy.Publisher("/stick_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
    
    def handle_whycon(self, msg):
        for i in msg.poses:
            stick_pose_or= geometry_msgs.msg.PoseStamped()
            stick_pose_or.header=msg.header
            stick_pose_or.pose=i

            #self.tf.waitForTransform(stick_pose_or.header.frame_id, "/base_link",stick_pose_or.header.stamp, rospy.Duration(4.0))
            stick_pose_d= self.tf.transformPose('/base_footprint', stick_pose_or)
            #stick_pose_d.pose.position.x= stick_pose_d.pose.position.x-0.12
            stick_pose_d.pose.orientation.x= 0.5
            stick_pose_d.pose.orientation.y= 0.5
            stick_pose_d.pose.orientation.z= 0.5
            stick_pose_d.pose.orientation.w= -0.5
            self.stick_pose_pub.publish(stick_pose_d)


if __name__ == '__main__':
    teleop = TransformS()
    rospy.spin()
