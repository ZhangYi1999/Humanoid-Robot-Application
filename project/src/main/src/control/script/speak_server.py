#!/usr/bin/env python
import rospy
import time
import sys
from control.srv import Speak,SpeakResponse
import almath
from naoqi import ALProxy

speakProxy = 0
motionProxy = 0
posProxy = 0


def service(req):
    if req.with_motion:
        isAbsolute = True

        if req.motion_name == "deny":
            names      = "HeadYaw"
            angleLists = [20.0*almath.TO_RAD, -20.0*almath.TO_RAD, 0.0]
            timeLists  = [0.25, 0.75, 1.0]
            motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        elif req.motion_name == "agree":
            names      = "HeadPitch"
            angleLists = [-10.0*almath.TO_RAD, 15.0*almath.TO_RAD, 0.0]
            timeLists  = [0.25, 0.75, 1.0]
            motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        elif req.motion_name == "up":
            names      = "RArm"
            target     = [0.174998,-0.037849,0.047863,3.051657,3.051657,3.051657]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names,motionProxy.FRAME_TORSO,target,max_speed,mask)
        elif req.motion_name == "down":
            names      = "RArm"
            target     = [0.032504,-0.114791,-0.113816,1.465496,1.465496,1.465496]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names,motionProxy.FRAME_TORSO,target,max_speed,mask)
        elif req.motion_name == "stand":
            posProxy.goToPosture("Stand",5)
        elif req.motion_name == "Sit":
            posProxy.goToPosture("Sit",5)
  
        
    speakProxy.say(req.message)
    return SpeakResponse(True)

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])

    speakProxy = ALProxy("ALTextToSpeech",robotIP,PORT)
    posProxy = ALProxy("ALRobotPosture",robotIP,PORT)
    motionProxy = ALProxy("ALMotion",robotIP,PORT)
    motionProxy.wakeUp()

    posProxy.goToPosture("Stand",2)
   
    rospy.init_node('speak_server')
    rospy.Service("speak_service", Speak, service)
    rospy.spin()