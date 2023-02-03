#!/usr/bin/env python3

import os
import face_recognition
import numpy as np
import sys
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray

num = 0

def img_callback(data):

    # declare name_msg which should contain names in the picture
    name_msg = Int8MultiArray()
    name_msg.data = []

    br = CvBridge()
    frame = br.imgmsg_to_cv2(data, "bgr8")

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_frame = frame[:, :, ::-1]
    # Find all the faces and face enqcodings in the frame of video
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
    # Loop through each face in this frame of video
    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # See if the face is a match for the known face(s)
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

        name = "Unknown"

        # If a match was found in known_face_encodings, just use the first one.
        # if True in matches:
        #     first_match_index = matches.index(True)
        #     name = known_face_names[first_match_index]

        # Or instead, use the known face with the smallest distance to the new face
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]
        
        if name == "song":
            name_msg.data.append(1)
        else:
            name_msg.data.append(0)

        # if name == 'ma':
        #     name_msg.data.append(1)
        # else:
        #     name_msg.data.append(0)

        # if name == 'yi':
        #     name_msg.data.append(1)
        # else:
        #     name_msg.data.append(0)
        
        # if name == 'chong':
        #     name_msg.data.append(1)
        # else:
        #     name_msg.data.append(0)

        name_pub.publish(name_msg)
        
        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        cv2.imshow('face', frame)
        cv2.waitKey(3)

if __name__ == '__main__':

    rospy.init_node('face_recognition')
    # create path for images
    dir_path = os.path.dirname(os.path.realpath(__file__))
    pkg_path = os.path.dirname(dir_path)
    syl_path = os.path.join(pkg_path, 'face_img/syl.jpeg')
    zcy_path = os.path.join(pkg_path, 'face_img/zcy.jpeg')
    # load images and detect the faces, save face encodings
    syl_image = face_recognition.load_image_file(syl_path)
    syl_face_encoding = face_recognition.face_encodings(syl_image)[0]
    zcy_image = face_recognition.load_image_file(zcy_path)
    zcy_face_encoding = face_recognition.face_encodings(zcy_image)[0]
    known_face_encodings = [
        syl_face_encoding,
        zcy_face_encoding
        ]
    known_face_names = [
        'song',
        'chong'
        ]

    image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, img_callback)
    name_pub = rospy.Publisher("/detected_human", Int8MultiArray, queue_size=10)

    cv2.namedWindow('face')

    rate = rospy.Rate (20)
    rospy.spin()