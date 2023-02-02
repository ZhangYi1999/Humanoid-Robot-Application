#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from control.srv import MoveJoints
from geometry_msgs.msg import Twist

motionProxy =0
posProxy = 0

def handle_move_joints(req):
    names = req.names
    frame = motion.FRAME_TORSO
    useSensorValues = False
    #angles = req.angles
    #fractionMaxSpeed = req.max_speed
    #timeLists = req.time_second

    # bring up the arm, in mode 1
    if (req.mode == 1):
        # nao should bring up or down arm
        target = [req.target.linear.x,req.target.linear.y,req.target.linear.z,req.target.angular.x,req.target.angular.y,req.target.angular.z]
        motionProxy.setPositions(names, frame, target, req.max_speed, req.mask)
        #motionProxy.setAngles(names,angles,fractionMaxSpeed)
    
    # deny
    elif (req.mode == 2):
        time_second = req.time_second
        angles = req.angles
        second_angles = req.second_angles
        isAbsolute = True
        motionProxy.angleInterpolation(names,angles,time_second,isAbsolute)
        time.sleep(0.2)
        motionProxy.angleInterpolation(names,second_angles,0.4,isAbsolute)
        time.sleep(0.4)
        motionProxy.angleInterpolation(names,0,time_second,isAbsolute)
        time.sleep(0.2)

    # agree
    elif (req.mode == 3):
        time_second = req.time_second
        second_names = req.second_names
        angles = req.angles
        second_angles = req.second_angles
        isAbsolute = True
        motionProxy.angleInterpolation(names,angles,time_second,isAbsolute)
        time.sleep(0.2)
        motionProxy.angleInterpolation(names,second_angles,time_second,isAbsolute)
        time.sleep(0.2)

    elif (req.mode == 4):
        if motionProxy.robotIsWakeUp and posProxy.getPosture is not "Sit":
            posProxy.goToPosture("Sit",4)
    
    elif (req.mode == 5):
        if motionProxy.robotIsWakeUp and posProxy.getPosture is not "Stand":
            posProxy.goToPosture("Stand",4)

    return True

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    motionProxy.setStiffnesses("Body" ,1.0)
    motionProxy.wakeUp()

    posProxy = ALProxy("ALRobotPosture",robotIP,PORT)
    if motionProxy.robotIsWakeUp and posProxy.getPosture is not "Stand":
        posProxy.goToPosture("Stand",2)

    rospy.init_node('move_joints_server')
    rospy.Service("move_joints", MoveJoints, handle_move_joints)
    rospy.spin()
			
		
