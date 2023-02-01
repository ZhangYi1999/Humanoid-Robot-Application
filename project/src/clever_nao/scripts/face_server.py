#!/usr/bin/env python
import rospy
import time
import sys
from clever_nao.srv import FaceDetect,FaceDetectResponse

def service(req):
    print("start")
    
    return True

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])

    rospy.init_node('which_server')
    rospy.Service("service_name", FaceDetect, service)
    rospy.spin()