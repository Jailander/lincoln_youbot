#!/usr/bin/env python

import sys
import rospy
import copy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import brics_actuator.msg

#/JointPositions

class Teleop(object):
    
    def __init__(self, linearAxisIndex = 1, angularAxisIndex = 0):
        rospy.init_node('picksie_teleop')
        self.init_moveit()
        rospy.on_shutdown(self._on_node_shutdown)
        
        self._joints=[]
        self.joint_names=['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']#, 'gripper_finger_joint_l', 'gripper_finger_joint_r']
        
        self.def_pos=[3.0, 2.61, -0.91, 0.25, 0.111]#2.88]
        self.drop_pos= [3.0, 0.03, -2.25, 0.25, 2.88]#, 0.0]


        self._xymode=False
        self.turbo = 1
        self.arm_disabled=True
        
        self._LinearAxisIndex = rospy.get_param("~linearAxisIndex", linearAxisIndex)
        self._AngularAxisIndex = rospy.get_param("~angularAxisIndex", angularAxisIndex)
        self._LinearScalingFactor = rospy.get_param("~linearScalingFactor", 0.5)
        self._AngularScalingFactor = rospy.get_param("~angularScalingFactor", 0.5)
        
        rospy.loginfo("Starting teleop node with linear axis %d and angular axis %d" % (self._LinearAxisIndex, self._AngularAxisIndex))
        

        for i in self.joint_names:
            joint={}
            joint['name']=i
            self._joints.append(joint)
            
        joint={}
        joint['name']= 'gripper_finger_joint_r'
        self._joints.append(joint)

        #print self._joints

        # subscriptions
        rospy.Subscriber("/joint_states", JointState, self._HandleJointStateMessage)
        rospy.Subscriber("/joy", Joy, self._HandleJoystickMessage)

        self._BricsCmdPublisher = rospy.Publisher("/arm_1/arm_controller/position_command", brics_actuator.msg.JointPositions, queue_size=1)
        self._GripperCmdPublisher = rospy.Publisher("/arm_1/gripper_controller/position_command", brics_actuator.msg.JointPositions, queue_size=1)
                
        self._VelocityCommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.sleep(2)
        self.go_to_search_pos()


    def init_moveit(self):
        moveit_commander.roscpp_initialize(sys.argv)        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm_1")
        print "============ Planning frame: %s" % self.group.get_planning_frame()
        print "============ effector frame: %s" % self.group.get_end_effector_link()
        print "============ Robot Groups:"
        print self.robot.get_group_names()
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============"


    def go_to_search_pos(self):
        print "going to search pos"
        self.group.clear_pose_targets()
        group_variable_values = self.def_pos
        self.group.set_joint_value_target(group_variable_values)
        
        plan2 = self.group.plan()
        self.group.go(wait=True)
        rospy.sleep(0.5)
        self.toggle_gripper(state=1)

#        for i in range(0,len(self.joint_names)):
#            arm_cmd = brics_actuator.msg.JointPositions()
#            j_cmd= brics_actuator.msg.JointValue()
#            j_cmd.joint_uri = self.joint_names[i]
#            j_cmd.unit = 'rad'
#            j_cmd.value = self.def_pos[i]
#            arm_cmd.positions.append(j_cmd)
#            print arm_cmd
#            self._BricsCmdPublisher.publish(arm_cmd)
#            rospy.sleep(1)
#        self.arm_disabled=False
        print "Done"

    def harvest(self):
        print "picking"
        waypoints = []
        # start with the current pose
        waypoints.append(self.group.get_current_pose().pose)
        
        # first orient gripper and move forward (+x)
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = 1.0
        wpose.position.x = waypoints[0].position.x# + 0.1
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z -0.05
        waypoints.append(copy.deepcopy(wpose))

        # second move down
        wpose.position.x += 0.08
        waypoints.append(copy.deepcopy(wpose))
        
        # second move down
        wpose.position.z += 0.03
        waypoints.append(copy.deepcopy(wpose))

        (plan3, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
        self.group.go(wait=True)
        rospy.sleep(0.5)
        self.toggle_gripper(state=-1)
        print "done"
        
#        # second move down
#        wpose.position.x += 0.05
#        waypoints.append(copy.deepcopy(wpose))
#        
#        # third move to the side
#        wpose.position.y += 0.05
#        waypoints.append(copy.deepcopy(wpose))

    def navigate_to_target(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        pose=rospy.wait_for_message("/stick_pose", geometry_msgs.msg.PoseStamped, timeout=90)    
        print "Requesting Navigation to "
        print pose

        movegoal = MoveBaseGoal()
        movegoal.target_pose.header.frame_id = "map"
        movegoal.target_pose.header.stamp = rospy.get_rostime()
        movegoal.target_pose.pose = pose.pose
        movegoal.target_pose.pose.position.x = movegoal.target_pose.pose.position.x - 0.5

        client.cancel_all_goals()
        rospy.sleep(rospy.Duration.from_sec(1))
        #print movegoal
        client.send_goal(movegoal)
        client.wait_for_result()

        velocityCommand = Twist()
        velocityCommand.linear.x = 0.0
        velocityCommand.linear.y = 0.0
        velocityCommand.angular.z = 0.0
        self._VelocityCommandPublisher.publish(velocityCommand)

        ps = client.get_result()  # A FibonacciResult
        self.arm_disabled=False
        print ps


    def go_to_drop_pos(self):
        print "going to drop pos"
        self.group.clear_pose_targets()
        group_variable_values = self.drop_pos
        self.group.set_joint_value_target(group_variable_values)
        
        self.plan2 = self.group.plan()
        self.group.go(wait=True)
        rospy.sleep(0.5)
        self.toggle_gripper(state=1)
#        for i in range(0,len(self.joint_names)):
#            arm_cmd = brics_actuator.msg.JointPositions()
#            j_cmd= brics_actuator.msg.JointValue()
#            j_cmd.joint_uri = self.joint_names[i]
#            j_cmd.unit = 'rad'
#            j_cmd.value = self.drop_pos[i]
#            arm_cmd.positions.append(j_cmd)
#            #print arm_cmd
#            self._BricsCmdPublisher.publish(arm_cmd)
#            rospy.sleep(1)
#        self.arm_disabled=False
        print "Done"

    def toggle_gripper(self, state=0):
        print self._joints[5]
        grp_cmd = brics_actuator.msg.JointPositions()
        j_cmd= brics_actuator.msg.JointValue()
        j_cmd.joint_uri = self._joints[5]['name']
        j_cmd.unit = 'm'
        if state == 0:
            if self._joints[5]['pos'] < 0.005:
                j_cmd.value = 0.0115
            else:
                j_cmd.value = 0.0
        else:
            if state == 1:
                j_cmd.value = 0.0115
            else:
                j_cmd.value = 0.0
                
        grp_cmd.positions.append(j_cmd)
        print "sending Gripper command %f" %j_cmd.value
        self._GripperCmdPublisher.publish(grp_cmd)


    def _HandleJointStateMessage(self, jointMessage):
        for i in self._joints:
            if i['name'] in jointMessage.name:
                i['pos'] = jointMessage.position[jointMessage.name.index(i['name'])]
                
        #print self._joints
    
    def _HandleJoystickMessage(self, joyMessage):
        turbo = self.turbo
        if joyMessage.buttons[9]:
            self._xymode = not self._xymode

        if joyMessage.buttons[6]:
            self.turbo = 1.0
            print "Turbo = %d" %turbo
        if joyMessage.buttons[7]:
            self.turbo += 1.0
            print "Turbo = %d" %turbo

        if joyMessage.buttons[4]:
            if not self._xymode:
                velocityCommand = Twist()
                velocityCommand.linear.x = self._LinearScalingFactor * joyMessage.axes[self._LinearAxisIndex]
                velocityCommand.linear.y = self._LinearScalingFactor * joyMessage.axes[3]
                velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[self._AngularAxisIndex]    		
            else:
                velocityCommand = Twist()
                velocityCommand.linear.x = self._LinearScalingFactor * joyMessage.axes[self._LinearAxisIndex]
                velocityCommand.linear.y = self._LinearScalingFactor * joyMessage.axes[self._AngularAxisIndex]
                velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[3]
        else:
            velocityCommand = Twist()
            velocityCommand.linear.x = 0.0
            velocityCommand.linear.y = 0.0
            velocityCommand.angular.z = 0.0
        self._VelocityCommandPublisher.publish(velocityCommand)
        
        if joyMessage.buttons[3]:
            self.go_to_search_pos()
            self.arm_disabled=True
            
        if joyMessage.buttons[0]:
            self.go_to_drop_pos()
            self.arm_disabled=True

        if joyMessage.buttons[1]:
            self.navigate_to_target()
            self.arm_disabled=True

        if joyMessage.buttons[2]:
            self.harvest()
            self.arm_disabled=True

        if joyMessage.buttons[5]:
            print self._joints
            self.toggle_gripper()

        
        if not self.arm_disabled:
            arm_cmd = brics_actuator.msg.JointPositions()
            
            if abs(joyMessage.axes[6])>0.5:
                j_cmd= brics_actuator.msg.JointValue()
                if joyMessage.axes[6] > 0:
                    j_cmd.joint_uri = self._joints[0]['name']
                    j_cmd.unit = 'rad'
                    j_cmd.value = self._joints[0]['pos'] + (0.0174533*turbo)
                    if j_cmd.value > 5.84 :
                       j_cmd.value = 5.84
                else:
                    j_cmd.joint_uri = self._joints[0]['name']
                    j_cmd.unit = 'rad'
                    j_cmd.value = self._joints[0]['pos'] - (0.0174533*turbo)
                    if j_cmd.value < 0.011 :
                       j_cmd.value = 0.011
                arm_cmd.positions.append(j_cmd)
    
            if abs(joyMessage.axes[7])>0.5:
                j_cmd= brics_actuator.msg.JointValue()
                if joyMessage.axes[7] > 0:
                    j_cmd.joint_uri = self._joints[1]['name']
                    j_cmd.unit = 'rad'
                    j_cmd.value = self._joints[1]['pos'] + (0.0174533*turbo)
                    if j_cmd.value > 2.617 :
                       j_cmd.value = 2.617 
                else:
                    j_cmd.joint_uri = self._joints[1]['name']
                    j_cmd.unit = 'rad'
                    j_cmd.value = self._joints[1]['pos'] - (0.0174533*turbo)
                    if j_cmd.value < 0.011 :
                       j_cmd.value = 0.011
                arm_cmd.positions.append(j_cmd)
    
    
            if joyMessage.buttons[0]:
                j_cmd= brics_actuator.msg.JointValue()
                j_cmd.joint_uri = self._joints[4]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = 2.90
                arm_cmd.positions.append(j_cmd)
    
    
#            if abs(joyMessage.axes[3])>0.5:
#                j_cmd= brics_actuator.msg.JointValue()
#                if joyMessage.axes[3] > 0:
#                    j_cmd.joint_uri = self._joints[4]['name']
#                    j_cmd.unit = 'rad'
#                    j_cmd.value = self._joints[4]['pos'] + 0.174533
#                    if j_cmd.value > 5.64 :
#                       j_cmd.value = 5.64
#                else:
#                    j_cmd.joint_uri = self._joints[4]['name']
#                    j_cmd.unit = 'rad'
#                    j_cmd.value = self._joints[4]['pos'] - 0.174533
#                    if j_cmd.value < 0.111 :
#                       j_cmd.value = 0.111
#                arm_cmd.positions.append(j_cmd)
    
            if abs(joyMessage.axes[4])>0.5:
                j_cmd= brics_actuator.msg.JointValue()
                if joyMessage.axes[4] > 0:
                    j_cmd.joint_uri = self._joints[3]['name']
                    j_cmd.unit = 'rad'
                    j_cmd.value = self._joints[3]['pos'] + (0.0174533 * abs(joyMessage.axes[4]))
                    if j_cmd.value > 3.42 :
                       j_cmd.value = 3.42 
                else:
                    j_cmd.joint_uri = self._joints[3]['name']
                    j_cmd.unit = 'rad'
                    j_cmd.value = self._joints[3]['pos'] - (0.0174533 * abs(joyMessage.axes[4]))
                    if j_cmd.value < 0.0222 :
                       j_cmd.value = 0.0222
                arm_cmd.positions.append(j_cmd)
    
    
            if joyMessage.axes[5] < 0.0:
                j_cmd= brics_actuator.msg.JointValue()
                j_cmd.joint_uri = self._joints[2]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[2]['pos'] + 0.0174533
                if j_cmd.value > -0.0158 :
                   j_cmd.value = -0.0158
                arm_cmd.positions.append(j_cmd)
    
            if joyMessage.axes[2] < 0.0:
                j_cmd= brics_actuator.msg.JointValue()
                j_cmd.joint_uri = self._joints[2]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[2]['pos'] - 0.0174533
                if j_cmd.value < -5.0655 :
                   j_cmd.value = -5.0655 
                arm_cmd.positions.append(j_cmd)
    
    
            if len(arm_cmd.positions) >0:
                self._BricsCmdPublisher.publish(arm_cmd)            
                            
    def _on_node_shutdown(self):
        print "bye"
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    teleop = Teleop()
    rospy.spin()