#!/usr/bin/env python
import rospy
import time
import motion
import numpy
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import MoveJoints
motionProxy =0;

def handle_move_joints(req):
    name = req.name
    frame = motion.FRAME_TORSO
    useSensorValues = False

####### if client ask current joint position, get it and save it in req.position
    if (req.task == 1):
        result = motionProxy.getPosition(name, frame, useSensorValues)
        req.current = result
        return req.current

    if (req.task == 2):
        motionProxy.wakeUp()
        # check which methode user choose and run.
        if req.max_speed is not None:
            motionProxy.setPositions(name, frame, req.target, req.max_speed, req.mask)
            time.sleep(2)
            
        elif req.time is not None:
            path = []
            target = almath.Position6D(req.target)
            path.append(list(target.toVector()))
            motionProxy.positionInterpolations(name, frame, path,req.mask, req.time)
        
      

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    rospy.Service("move_joints", MoveJoints, handle_move_joints)
    rospy.spin()
			
		
