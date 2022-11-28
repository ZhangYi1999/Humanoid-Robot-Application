-#!/usr/bin/env python
import tf
import cv2
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_2.srv import MoveJoints
import argparse 
from geometry_msgs.msg import Twist 

motionProxy =0


# Bonus mode switching 

#Bonus = True
Bonus = False


def handle_move_joints(req):
    name = req.name
    frame = motion.FRAME_TORSO
    useSensorValues = True
    result = motionProxy.getPosition(name, frame, useSensorValues)
    pose6D=Twist()
    # Vector3  linear & Vector3  angular
    pose6D.linear.x=result[0]  
    pose6D.linear.y=result[1]
    pose6D.linear.z=result[2]
    pose6D.angular.x=result[3]
    pose6D.angular.y=result[4]
    pose6D.angular.z=result[5]
    req.current = result
    return pose6D, req.current


def movejoints(req):
    for i in range(10):
        motionProxy.setStiffnesses('Body',1)
    
    name = req.name
    frame = motion.FRAME_TORSO
    useSensorValues = True
    result = motionProxy.getPosition(name, frame, useSensorValues)
    pose6D=Twist()
    position = [0, 0, 0, 0, 0, 0]
    Name = req.name
    frame = motion.FRAME_TORSO
    fractionMaxSpeed = req.velocity

    if req.max_speed is not None:
    # Axis mask. 7 for position only, 
    # 56 for rotation only 
    # 63 for position and rotation
        axisMask = 63 
        position[0] = req.desired.linear.x
        position[1] = req.desired.linear.y
        position[2] = req.desired.linear.z
        position[3] = req.desired.angular.x
        position[4] = req.desired.angular.y
        position[5] = req.desired.angular.z
        req.current = motionProxy.setPositions(Name, frame, position, fractionMaxSpeed, axisMask)   # Exercise 1.2 1.3

    # not quite sure, whether it needs for 1.4

    elif req.time is not None:
        dx = 0.03 # translation axis X (meters)
        dy = 0.04 # translation axis Y (meters)
        pathList     = []    
        axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
        timeList     = [[1.0], [1.0]]         # seconds
        Name.append("LArm")
        currentPos = motionProxy.getPosition("LArm", frame, useSensorValues)
        targetPos = almath.Position6D(currentPos)
        targetPos.y -= dy
        pathList.append(list(targetPos.toVector()))

        Name.append("RArm")
        currentPos = motionProxy.getPosition("RArm", frame, useSensorValues)
        targetPos = almath.Position6D(currentPos)
        targetPos.y += dy
        pathList.append(list(targetPos.toVector()))

        req.current = motionProxy.positionInterpolations(Name, frame, pathList, axisMaskList, timeList)
    
    return 1, req.current


def movejoints_aruco(req):
    for i in range(10):
        motionProxy.setStiffnesses('Body',1)
    # initial but changable
    positionLArm = [0, 0, 0.25, 0, 0, 0]  # LArm position 
    positionRArm = [0, 0, -0.25, 0, 0, 0]   

    if req.desired.linear.z==0:
        motionProxy.setPositions("LArm", motion.FRAME_TORSO, positionLArm, 1.0, 63)
        motionProxy.setPositions("RArm", motion.FRAME_TORSO, positionRArm, 1.0, 63)

    else:
        frame     = motion.FRAME_TORSO
        fractionMaxSpeed = 1.0
        # 7 for only control position
        axisMask = 7 
        #Frame: top optical frame --> carmera frame --> quad.: homogen trandsformation
        position0 = np.array([[req.desired.linear.x], [req.desired.linear.y], [req.desired.linear.z]]) 
        position1 = np.array([[0.0], [0.0], [0.0]]) 
        hom_position2 = np.array([[0.0], [0.0], [0.0] ,[0.0]]) 

        position2_RArm = [0.0, 0.0, 0.0, 1.23374, -0.904337, 1.69707]  # LArm position 
        position2_LArm = [0.0, 0.0, 0.0, -1.69423, -0.487957,-1.37036]  # RArm position

        # optical frame to camera frame
        rotation = np.array([[0.0,-1.0,0.0],[0.0,0.0,-1.0],[1.0,0.0,0.0]])
        position1 = (rotation.T).dot(position0)

        name  = 'CameraTop'
        useSensorValues  = True
        result = motionProxy.getTransform(name, frame, useSensorValues)
        result_np = np.arange(16.0)

        for i in range(0, 16):
            result_np[i] =  result[i]
        
        trans_tor2cam = np.reshape(result, (4, 4))
        hom_position1 = np.array([[0.0],[0.0],[0.0],[1.0]])
        hom_position1[0:3] = position1 # R p 0
        hom_position2 = trans_tor2cam.dot(hom_position1)
        hom_position2 = np.reshape(hom_position2, 4)
        rotation_cam2optical = np.array([[0.19703548,0.56251667,0.80296452],
                                        [-0.56251667, -0.60592903,  0.56251667],
                                        [ 0.80296452, -0.56251667,  0.19703548]])
        rotation_tor2optical = trans_tor2cam[0:3,0:3].dot(rotation_cam2optical)
        rotation_opt2marker_vec= np.array([[req.desired.angular.x], [req.desired.angular.y], [req.desired.angular.z]])
        rotation_tuple = cv2.Rodrigues(rotation_opt2marker_vec)
        rotation_opt2marker_mat = rotation_tuple[0]
        rotation_tor2marker_mat = rotation_tor2optical.dot(rotation_opt2marker_mat)
        rotation_tuple1 = cv2.Rodrigues(rotation_tor2marker_mat)
        rotation_tor2marker_vec = rotation_tuple1[0]
        rotation_tor2marker_vec = np.reshape(rotation_tor2marker_vec, 3)

        # move with setPositions
        if req.desired.linear.x>=0:
            position2_RArm[0] = hom_position2[0]
            position2_RArm[1] = hom_position2[1]
            position2_RArm[2] = hom_position2[2]
            if Bonus==True:
                position2_RArm[3] = rotation_tor2marker_vec[0]
                position2_RArm[4] = rotation_tor2marker_vec[1]
                position2_RArm[5] = rotation_tor2marker_vec[2]
                axisMask=63
            position2_RArm[3] = rotation_tor2marker_vec[0]
            position2_RArm[4] = rotation_tor2marker_vec[1]
            position2_RArm[5] = rotation_tor2marker_vec[2]
            motionProxy.setPositions("RArm", frame, position2_RArm, fractionMaxSpeed, axisMask)
            motionProxy.setPositions("LArm", motion.FRAME_TORSO, positionLArm, 1.0, 63)

        else:
            position2_LArm[0] = hom_position2[0]
            position2_LArm[1] = hom_position2[1]
            position2_LArm[2] = hom_position2[2]
            if Bonus==True:
                position2_LArm[3] = rotation_tor2marker_vec[0]
                position2_LArm[4] = rotation_tor2marker_vec[1]
                position2_LArm[5] = rotation_tor2marker_vec[2]
                axisMask=63
            motionProxy.setPositions("LArm", frame, position2_LArm, fractionMaxSpeed, axisMask)
            motionProxy.setPositions("RArm", motion.FRAME_TORSO, positionRArm, 1.0, 63)
    return 1

def cartesian_coordinate_server():

    #.service(name,servicetype(*.srv),callbackfunction)
    s1 = rospy.Service('coordinate', coordinate, joint_cartesian)
    s2 = rospy.Service('MoveJoint', MoveJoints, movejoints)
    s3 = rospy.Service('MoveJoint_aruco', MoveJoints, movejoints_aruco)


if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')
    rospy.Service("move_joints", MoveJoints, handle_move_joints)

    cartesian_coordinate_server()
    rospy.spin()
			