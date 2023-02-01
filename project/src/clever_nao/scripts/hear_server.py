#!/usr/bin/env python
import rospy
import time
import sys
from clever_nao.srv import Hear,HearResponse
from naoqi import ALProxy

speechRecogProxy = 0
memoryProxy = 0

def service(req):
    name = ""
    question = ""

    if req.mode is "set name":
        speechRecogProxy.setVocabulary(req.names,False)
    elif req.mode is "hear name":
        speechRecogProxy.subscribe("Listener")

        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while True:
            val = memoryProxy.getData("WordRecognized")
            if val[0] is not "":
                
                    
                print(val[0])
            
            if rospy.Time.now().to_sec() - start > 10:
                break
            rate.sleep()

        speechRecogProxy.unsubscribe("Listener")
    elif req.mode is "hear question":
        word_list = ["When"]
        speechRecogProxy.setVocabulary(word_list,False)
        speechRecogProxy.subscribe("Listener")

        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while True:
            val = memoryProxy.getData("WordRecognized")
            if val[0] is not "":
                
                    
                print(val[0])
            
            if rospy.Time.now().to_sec() - start > 10:
                break
            rate.sleep()

        speechRecogProxy.unsubscribe("Listener")
    else:
        word_list = ["Nao"]
        speechRecogProxy.setVocabulary(word_list,False)
        speechRecogProxy.subscribe("Listener")

        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while True:
            val = memoryProxy.getData("WordRecognized")
            if val[0] is not "":
                
                    
                print(val[0])
            
            if rospy.Time.now().to_sec() - start > 10:
                break
            rate.sleep()

        speechRecogProxy.unsubscribe("Listener")

    
    return HearResponse(name,question)

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])

    

    speechRecogProxy = ALProxy("ALSpeechRecognition",robotIP,PORT)
    speechRecogProxy.unsubscribe("Listener")
    
    
    memoryProxy = ALProxy("ALMemory",robotIP,PORT)

    rospy.init_node('hear_server')
    rospy.Service("hear_service", Hear, service)
    rospy.spin()