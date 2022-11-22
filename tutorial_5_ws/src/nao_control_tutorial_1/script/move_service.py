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
    #Exercise 1
    motionProxy.setStiffnesses("RArm" ,1.0)
    names = 'RShoulderRoll'
    angles = 30.0*almath.TO_RAD
    fractionMaxSpeed = 0.1
    motionProxy.setAngles(names,angles,fractionMaxSpeed)
    motionProxy.setStiffnesses("RArm", 0.0)


    #Exercise 2
    motionProxy.setStiffnesses("LArm" ,1.0)
    names = AL::ALValue::array("LShoulderRoll", "LShoulderPitch")
    angles = [30.0*almath.TO_RAD, 0.0*almath.TO_RAD]
    timeLists = [2.0, 1.]
    bool isAbsolute = true;
    motionProxy.angleInterpolation(names,angles,fractionMaxSpeed,isAbsolute)
    motionProxy.setStiffnesses("LArm", 0.0)



    # if req.id==1:
    #     for i in range(10):
    #         motionProxy.setStiffnesses("Body" ,1.0)
    #     motionProxy.setAngles(req.name,req.angle,req.speed)
    #     print("Ready")
    # if req.id==2:        
    #     for i in range(30):
    #         motionProxy.setStiffnesses("Body" ,1.0)
    #     name = req.name
    #     angle = req.angle
    #     time=req.time
    #     isAbsolute=True
    #     motionProxy.angleInterpolation(name,angle,time,isAbsolute)
    #     list = motionProxy.getTaskList()

    #     if len(list) == 1:
    #         motionProxy.killTask(list[0][1])
    #     if len(list) == 2:
    #         motionProxy.killTask(list[1][1])
    #     print("Ready to move with interpolation")


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    service = rospy.Service("move_joints", MoveJoints, handle_move_joints)
    
    rospy.spin()
			
		
