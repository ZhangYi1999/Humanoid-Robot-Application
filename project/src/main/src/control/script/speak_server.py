#!/usr/bin/env python
import rospy
import time
import sys
import motion
from control.srv import Speak,SpeakResponse
import almath
from naoqi import ALProxy

speakProxy = 0
motionProxy = 0
posProxy = 0


def service(req):
    if req.with_motion:
        isAbsolute = True

        # this predefined motion, input will be the motion name and
        # the word, that want nao to talk, return true if the process success
        # shake head
        if req.motion_name == "deny":
            names      = "HeadYaw"
            angleLists = [20.0*almath.TO_RAD, -20.0*almath.TO_RAD, 0.0]
            timeLists  = [0.25, 0.75, 1.0]
            motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        # nod head
        elif req.motion_name == "agree":
            names      = "HeadPitch"
            angleLists = [-10.0*almath.TO_RAD, 15.0*almath.TO_RAD, 0.0]
            timeLists  = [0.25, 0.75, 1.0]
            motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        #bring up arm
        elif req.motion_name == "up":
            names      = "RArm"
            target     = [0.174998,-0.037849,0.047863,3.051657,3.051657,3.051657]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names,motion.FRAME_TORSO,target,max_speed,mask)
        # put down arm
        elif req.motion_name == "down":
            names      = "RArm"
            target     = [0.032504,-0.114791,-0.113816,1.465496,1.465496,1.465496]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names,motion.FRAME_TORSO,target,max_speed,mask)
        
        elif req.motion_name == "wait":
            names_r    = "RArm"
            target_r   = [0.054214,-0.052757,-0.048563,1.266628,1.266628,1.266628]
            names_l    = "LArm"
            target_l   = [0.058802,0.044634,-0.059294,-1.602201,-1.602201,-1.602201]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names_l,motion.FRAME_TORSO,target_l,max_speed,mask)
            motionProxy.post.setPositions(names_r,motion.FRAME_TORSO,target_r,max_speed,mask)

        elif req.motion_name == "welcome":
            names_l   = "LArm"
            target_l  = [0.143298,0.214727,0.093595,-2.782163,-2.782163,-2.782163]
            names_r   = "RArm"
            target_r  = [0.152911,-0.211671,0.081617,2.772783,2.772783,2.772783]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names_l,motion.FRAME_TORSO,target_l,max_speed,mask)
            motionProxy.post.setPositions(names_r,motion.FRAME_TORSO,target_r,max_speed,mask)
        
        elif req.motion_name == "where":
            names      = "HeadYaw"
            angleLists = [70.0*almath.TO_RAD, -70.0*almath.TO_RAD, 0.0]
            timeLists  = [0.4, 1.2, 1.6]
            motionProxy.post.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        elif req.motion_name == "angry":
            names_l   = "LArm"
            target_l  = [-0.002289,0.100365,-0.054560,-2.999208,-2.999208,-2.999208]
            names_r   = "RArm"
            target_r  = [0.007664,-0.091994,-0.041811,2.702231,2.702231,2.702231]
            max_speed  = 1.0
            mask       = 7
            motionProxy.post.setPositions(names_l,motion.FRAME_TORSO,target_l,max_speed,mask)
            motionProxy.post.setPositions(names_r,motion.FRAME_TORSO,target_r,max_speed,mask)

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
