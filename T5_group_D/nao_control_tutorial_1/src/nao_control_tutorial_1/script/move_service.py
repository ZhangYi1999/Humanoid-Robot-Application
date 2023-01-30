#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_1.srv import MoveJoints
motionProxy =0

#TODO: create service handler
def handle_move_joints(req):
    names = req.names
    angles = req.angles
    fractionMaxSpeed = req.max_speed
    timeLists = req.time_second
    isAbsolute = True

    if(req.mode == 1):
        motionProxy.setStiffnesses("Body" ,1.0)
        motionProxy.setAngles(names,angles,fractionMaxSpeed)
        time.sleep(2)

    elif(req.mode == 2):
        motionProxy.setStiffnesses("Body" ,1.0)
        motionProxy.angleInterpolation(names,angles,timeLists,isAbsolute)
        motionProxy.setStiffnesses("Body", 0.0)
        taskList = motionProxy.getTaskList()
        uiMotion = taskList[0][1]
        motionProxy.killTask(uiMotion)

    return True


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    rospy.Service("move_joints", MoveJoints, handle_move_joints)
    
    rospy.spin()
			
		
