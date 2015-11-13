#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import geometry_msgs.msg
from geometry_msgs.msg import Twist

from move_base_msgs.msg import *

class nav_client(object):
    
    def __init__(self) :
        
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._VelocityCommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
    

        pose=rospy.wait_for_message("/stick_pose", geometry_msgs.msg.PoseStamped, timeout=90)    
        print "Requesting Navigation to "
        print pose

        movegoal = MoveBaseGoal()
        movegoal.target_pose.header.frame_id = pose.header.frame_id
        movegoal.target_pose.header.stamp = rospy.get_rostime()
        movegoal.target_pose.pose = pose.pose
        movegoal.target_pose.pose.position.x = movegoal.target_pose.pose.position.x - 0.5
#        .position.x = float(inf[0])
#        movegoal.target_pose.pose.position.y = float(inf[1])
        movegoal.target_pose.pose.orientation.x = 0
        movegoal.target_pose.pose.orientation.y = 0
        movegoal.target_pose.pose.orientation.z = 0.0#float(inf[5])
        movegoal.target_pose.pose.orientation.w = 1.0#float(inf[6])
        self.client.cancel_all_goals()
        rospy.sleep(rospy.Duration.from_sec(1))
        #print movegoal
        self.client.send_goal(movegoal)
        self.client.wait_for_result()

        velocityCommand = Twist()
        velocityCommand.linear.x = 0.0
        velocityCommand.linear.y = 0.0
        velocityCommand.angular.z = 0.0
        self._VelocityCommandPublisher.publish(velocityCommand)

        ps = self.client.get_result()  # A FibonacciResult
        print ps

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('nav_test')
    ps = nav_client()
    
