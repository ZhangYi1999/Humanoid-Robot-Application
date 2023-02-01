#!/usr/bin/env python
import rospy
import time
import sys
from clever_nao.srv import Speak,SpeakResponse
from naoqi import ALProxy

speakProxy = 0
motionProxy = 0
posProxy = 0
motionspeakProxy = 0

def service(req):
    if req.with_motion:

        if motionProxy.robotIsWakeUp and posProxy.getPosture is not "Stand":
            posProxy.goToPosture("Stand",2)
        
        words = ""

        if req.motion_name is "Reject":
            words = "^start(animations/Stand/Gestures/No_8)" + req.message + "^wait(animations/Stand/Gestures/No_8)"
        elif req.motion_name is "Pass":
            words = "^start(animations/Stand/Gestures/Yes_2)" + req.message + "^wait(animations/Stand/Gestures/Yes_2)"
        elif req.motion_name is "Nothing":
            words = "^start(animations/Stand/Gestures/YouKnowWhat_1)" + req.message + "^wait(animations/Stand/Gestures/YouKnowWhat_1)"
        elif req.motion_name is "Welcome":
            words = "^start(animations/Stand/Gestures/Hey_6)" + req.message + "^wait(animations/Stand/Gestures/Hey_6)"
        elif req.motion_name is "Explain":
            words = "^start(animations/Stand/Gestures/Explain_3)" + req.message + "^wait(animations/Stand/Gestures/Explain_3)"
        else:
            words = "^start(animations/Stand/Gestures/BodyTalk_5)" + req.message + "^wait(animations/Stand/Gestures/BodyTalk_5)"
        
        motionspeakProxy.say(words)
        posProxy.goToPosture("Stand",2)
    else: 
        speakProxy.say(req.message)
    

    return SpeakResponse(True)

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])

    motionspeakProxy = ALProxy("ALAnimatedSpeech",robotIP,PORT)
    speakProxy = ALProxy("ALTextToSpeech",robotIP,PORT)

    posProxy = ALProxy("ALRobotPosture",robotIP,PORT)
    motionProxy = ALProxy("ALMotion",robotIP,PORT)

    stiffnessLists  = 0.0
    timeLists  = 1.0

    motionProxy.wakeUp()
    
    rospy.init_node('speak_server')
    rospy.Service("speak_service", Speak, service)
    rospy.spin()