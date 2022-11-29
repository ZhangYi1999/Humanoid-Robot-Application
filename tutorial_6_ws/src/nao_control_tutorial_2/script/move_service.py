#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import MoveJoints
from geometry_msgs.msg import Twist

motionProxy =0

def handle_move_joints(req):
    name = req.name
    frame = motion.FRAME_TORSO
    useSensorValues = False

####### if client ask current joint position, get it and save it in req.position
    if (req.task == 1):
        result = motionProxy.getPosition(name, frame, useSensorValues)
        current = Twist()
        current.linear.x = result[0]
        current.linear.y = result[1]
        current.linear.z = result[2]
        current.angular.x = result[3]
        current.angular.y = result[4]
        current.angular.z = result[5]
        return current

    if (req.task == 2):
        target = [req.target.linear.x,req.target.linear.y,req.target.linear.z,req.target.angular.x,req.target.angular.y,req.target.angular.z]
        motionProxy.wakeUp()
        # check which methode user choose and run.
        if req.max_speed is not None:
            print(target)
            motionProxy.setPositions(name, frame, target, req.max_speed, req.mask)
            time.sleep(2)
            
        elif req.time is not None:
            path = []
            target = almath.Position6D(req.target)
            path.append(list(target.toVector()))
            motionProxy.positionInterpolations(name, frame, path,req.mask, req.time)
            print("2")

    if (req.task == 3):
        target_position = np.array([[req.target.linear.x],[req.target.linear.y],[req.target.linear.z]])
        camera_position = np.array([[0.0],[0.0],[0.0]])
        
        R = np.array([[0.0,-1.0,0.0],[0.0,0.0,-1.0],[1.0,0.0,0.0]])
        # top optical to top camera
        target_in_camera = (R.T).dot(target_position)
        # transformation from camera to torso
        name = "CameraTop"
        camera_torso = motionProxy.getTransform(name, frame, useSensorValues)
        camera_torso_np = np.zeros(16)
        for i in range(16):
            camera_torso_np[i] = camera_torso[i]
        transform_tor_cam = np.reshape(camera_torso_np,(4,4))
        # homogen position of aruco in camera
        homo_pos_cam = np.array([[target_in_camera[0]],[target_in_camera[1]],[target_in_camera[2]],[1.0]])
        # homogen position of aruco in torso
        homo_pos_torso = transform_tor_cam.dot(homo_pos_cam)
        #print(np.shape(homo_pos_torso))
        homo_pos_torso = np.reshape(homo_pos_torso,4)

        right_name = "RHand"
        left_name = "LHand"

        # get transform of each hand base on torso frame
        left_trans = motionProxy.getTransform(left_name, frame, useSensorValues)
        right_trans = motionProxy.getTransform(right_name, frame, useSensorValues)

        # get the position of each point in torso
        aruco_pos_torso = np.array([[homo_pos_torso[3]],[homo_pos_torso[7]],[homo_pos_torso[11]]])
        left_pos_torso = np.array([[left_trans[3]],left_trans[7], left_trans[11]])
        right_pos_torso = np.array([[right_trans[3]], right_trans[7],right_trans[11]])

        # calculate the distance base on torso frame
        dist_aruco_left = np.sqrt(np.square(aruco_pos_torso - left_pos_torso))
        dist_aruco_right = np.sqrt(np.square(aruco_pos_torso - right_pos_torso))

        motionProxy.wakeUp()

        # initial position of each arm
        left_pos_init = motionProxy.getPosition("LArm", frame, useSensorValues)
        right_pos_init = motionProxy.getPosition("RArm", frame, useSensorValues)

        ori = {0.0,0.0,0.0}
        target_torso = {aruco_pos_torso[0],aruco_pos_torso[1],aruco_pos_torso[2],ori[0],ori[1],ori[2]}

        if dist_aruco_left > dist_aruco_right:
            # move right arm
            motionProxy.setPositions("RArm", frame, target_torso, req.max_speed, req.mask)
            motionProxy.setPositions("LArm", frame, left_pos_init, req.max_speed, req.mask)
        elif dist_aruco_left < dist_aruco_right:
            # move left arm
            motionProxy.setPositions("LArm", frame, target_torso, req.max_speed, req.mask)
            motionProxy.setPositions("RArm", frame, right_pos_init, req.max_speed, req.mask)
        else:
            # move both arm
            motionProxy.setPositions("RArm", frame, target_torso, req.max_speed, req.mask)
            motionProxy.setPositions("LArm", frame, target_torso, req.max_speed, req.mask)


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    rospy.Service("move_joints", MoveJoints, handle_move_joints)
    rospy.spin()
			
		