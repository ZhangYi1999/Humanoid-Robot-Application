#!/usr/bin/env python
import rospy
import time
import sys
from control.srv import Speech
from naoqi import ALProxy

speechRecogProxy = 0
memoryProxy = 0

word_list = ["When", "Long", "Many"]
# when: when will arrive munich
# long: how long will it take from current station to munich
# many: how many stations still have

def service(req):
    question == "no"
    if req.start == True:
        speechRecogProxy.setVocabulary(word_list,False)
        speechRecogProxy.subscribe("speech")
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while True:
            val = memoryProxy.getData("WordRecognized")
            if val is not "":
                question = val
                time.sleep(1)
                break

            if rospy.Time.now().to_sec() - start > 10:
                break
            rate.sleep()

    speechRecogProxy.unsubscribe("speech")
    return question
            


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    speechRecogProxy = ALProxy("ALSpeechRecognition",robotIP,PORT)
    speechRecogProxy.setLanguage("English")
    #speechRecogProxy.unsubscribe("speech")
    

    memoryProxy = ALProxy("ALMemory",robotIP,PORT)

    rospy.init_node('speech_service')
    rospy.Service("speech_service", Speech, service)
    rospy.spin()