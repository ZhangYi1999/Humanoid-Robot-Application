#!/usr/bin/env python3

import os
import face_recognition
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray

class face_detect:

    name_msg = Int8MultiArray()


    def __init__(self):
        self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.img_callback)
        self.name_pub = rospy.Publisher("/detected_human", Int8MultiArray, queue_size=10)
        self.bridge = CvBridge()
        
        rospy.loginfo("Initialize face detection.")

        # create path for images
        dir_path = os.path.dirname(os.path.realpath(__file__))
        pkg_path = os.path.dirname(dir_path)
        syl_path = os.path.join(pkg_path, 'face_img/syl.jpeg')
        zy_path = os.path.join(pkg_path, 'face_img/zy.jpeg')
        zcy_path = os.path.join(pkg_path, 'face_img/zcy.jpeg')
        # load images and detect the faces, save face encodings
        syl_image = face_recognition.load_image_file(syl_path)
        syl_face_encoding = face_recognition.face_encodings(syl_image)[0]
        zy_image = face_recognition.load_image_file(zy_path)
        zy_face_encoding = face_recognition.face_encodings(zy_image)[0]
        zcy_image = face_recognition.load_image_file(zcy_path)
        zcy_face_encoding = face_recognition.face_encodings(zcy_image)[0]
        self.known_face_encodings = [
            syl_face_encoding,
            zy_face_encoding,
            zcy_face_encoding
            ]
        self.known_face_names = [
            'song',
            'yi',
            'chong'
            ]
        rospy.loginfo("Load face data")
        print(self.known_face_encodings)

        self.num = 0

        


    def img_callback(self, data):
        if self.num % 10 == 0:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.name_msg.data.clear()

            # cv2.imshow('face', frame)
            # # cv2.resizeWindow('face', 640, 480)
            # cv2.waitKey(30)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_frame = frame[:, :, ::-1]

            # Find all the faces and face enqcodings in the frame of video
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
            
            # Loop through each face in this frame of video
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)

                name = "Unknown"

                # If a match was found in known_face_encodings, just use the first one.
                # if True in matches:
                #     first_match_index = matches.index(True)
                #     name = known_face_names[first_match_index]

                # Or instead, use the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
                
                if name == "song":
                    self.name_msg.data.append(1)
                else:
                    self.name_msg.data.append(0)

                # if name == 'yi':
                #     self.name_msg.data.append(1)
                # else:
                #     self.name_msg.data.append(0)

                # if name == 'ma':
                #     self.name_msg.data.append(1)
                # else:
                #     self.name_msg.data.append(0)
                
                # if name == 'chong':
                #     self.name_msg.data.append(1)
                # else:
                #     self.name_msg.data.append(0)

                self.name_pub.publish(self.name_msg)
                
                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 0.8, (255, 255, 255), 1)
                
                cv2.imshow('face', frame)
                cv2.waitKey(2)


        self.num = self.num + 1
        

if __name__ == '__main__':
    node = face_detect()
    rospy.init_node('face_detect', anonymous=False)
    rospy.spin()


