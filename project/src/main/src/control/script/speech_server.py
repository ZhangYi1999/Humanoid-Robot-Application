#!/usr/bin/env python
import rospy
import time
import sys
from control.srv import Speech,SpeechResponse
from naoqi import ALProxy

speechRecogProxy = 0
memoryProxy = 0
posProxy = 0

name_list = ["Jack","Mike","Amy","Tom","John"]
word_list = ["When","Many","North","South"]
# when: when will arrive hamburg
# long: how long will it take from current station to hamburg
# many: how many stations still have

def service(req):
    
    question = "no"
    name = "None"
    # mode 1 for answer the question, input will be the question from passenger
    # and returnn the key word from question
    if req.mode == 1:
        speechRecogProxy.setVocabulary(word_list,False)
        speechRecogProxy.subscribe("hear question")
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while True:
            val = memoryProxy.getData("WordRecognized")
            if val[0] is not "":
                question = val[0]
                time.sleep(1)
                break
            if rospy.Time.now().to_sec() - start > 10:
                break
            rate.sleep()
        speechRecogProxy.unsubscribe("hear question")
    
    # mode 2 for check the name from passenger, input will be the name from passenger voise
    # return name of passenger
    elif req.mode == 2:
        speechRecogProxy.setVocabulary(name_list,False)
        speechRecogProxy.subscribe("hear name")
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while True:
            val = memoryProxy.getData("WordRecognized")
            if val[0] is not "":
                name = val[0]
                break

            if rospy.Time.now().to_sec() - start > 10:
                break
            rate.sleep()
        speechRecogProxy.unsubscribe("hear name")
    
    posProxy.goToPosture("Stand",5)
    
    return SpeechResponse(question,name)
            


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    speechRecogProxy = ALProxy("ALSpeechRecognition",robotIP,PORT)
    speechRecogProxy.setLanguage("English")
    #speechRecogProxy.unsubscribe("speech")
    # speechRecogProxy.unsubscribe("hear question")
    

    memoryProxy = ALProxy("ALMemory",robotIP,PORT)
    posProxy = ALProxy("ALRobotPosture",robotIP,PORT)

    rospy.init_node('speech_service')
    rospy.Service("speech_service", Speech, service)
    rospy.spin()
