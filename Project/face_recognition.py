#!/usr/bin/env python

import os
import face_recognition
import numpy as np
import sys
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

num = 0

def img_callback(data):
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
        
        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        cv2.imshow('Video', frame)

if __name__ == '__main__':

    rospy.init_node('face_recognition')
    dir_path = os.getcwd()
    pkg_path = os.path.dirname(dir_path)
    syl_path = os.path.join(pkg_path, '/face_img/syl.jpg')
    syl_image = face_recognition.load_image_file(syl_path)
    syl_face_encoding = face_recognition.face_encodings(syl_image)[0]
    known_face_encodings = [
        syl_face_encoding
        ]
    known_face_names = [
        'song'
        ]

    image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, img_callback)

    rate = rospy.Rate (20)
    rospy.spin()