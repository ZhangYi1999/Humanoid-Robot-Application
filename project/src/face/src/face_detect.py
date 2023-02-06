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


    def __init__(self, input_known_face_encodings, input_known_face_names):
        self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.img_callback)
        self.name_pub = rospy.Publisher("/detected_human", Int8MultiArray, queue_size=10)
        self.bridge = CvBridge()

        self.known_face_encodings = input_known_face_encodings
        self.known_face_names = input_known_face_names

        self.name_msg.data.append(0)
        self.name_msg.data.append(0)
        self.name_msg.data.append(0)
        self.name_msg.data.append(0)

        self.count = 0
        self.face_history = np.zeros((5,4), dtype=int)
        
        rospy.loginfo("Initialize face detection.")


    def img_callback(self, data):
        # if self.num % 10 == 0:
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.name_msg.data[0] = 0
        self.name_msg.data[1] = 0
        self.name_msg.data[2] = 0
        self.name_msg.data[3] = 0

        # cv2.imshow('face', frame)
        # # cv2.resizeWindow('face', 640, 480)
        # cv2.waitKey(30)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_frame = frame[:, :, ::-1]

        # Find all the faces and face enqcodings in the frame of video
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        
        if face_encodings:

            # Loop through each face in this frame of video
            for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)

                name = "Unknown"

                # Use the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
                
                # if name == "Song":
                #     self.name_msg.data[0] = 1
                # else:
                #     self.name_msg.data[0] = 0

                # if name == 'Yi':
                #     self.name_msg.data[1] = 1
                # else:
                #     self.name_msg.data[1] = 0

                # if name == 'Ma':
                #     self.name_msg.data[2] = 1
                # else:
                #     self.name_msg.data[2] = 0
                
                # if name == 'Chong':
                #     self.name_msg.data[3] = 1
                # else:
                #     self.name_msg.data[3] = 0

                if name == "Mike":
                    self.face_history[self.count, 0] = 1
                else:
                    self.face_history[self.count, 0] = 0
                
                if name == "Jack":
                    self.face_history[self.count, 1] = 1
                else:
                    self.face_history[self.count, 1] = 0
                
                if name == "Tom":
                    self.face_history[self.count, 2] = 1
                else:
                    self.face_history[self.count, 2] = 0

                if name == "John":
                    self.face_history[self.count, 3] = 1
                else:
                    self.face_history[self.count, 3] = 0
                
                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 30), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 0.7, (255, 255, 255), 1)
        
        else:
            self.face_history[self.count, 0] = 0
            self.face_history[self.count, 1] = 0
            self.face_history[self.count, 2] = 0
            self.face_history[self.count, 3] = 0
        
        self.count = self.count + 1
        if self.count == 5:
            self.count = 0
        
        stats = np.sum(self.face_history, axis=0)
        # print(self.face_history)
        if np.sum(stats) != 0:
            index = np.argmax(stats)
            self.name_msg.data[index] = 1
        self.name_pub.publish(self.name_msg)

        cv2.imshow('face', frame)
        cv2.waitKey(2)

        # cv2.imshow('face', frame)
        # cv2.waitKey(2)
        # self.num = self.num + 1
        

if __name__ == '__main__':
    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    pkg_path = os.path.dirname(dir_path)
    syl_path = os.path.join(pkg_path, 'face_img/syl.jpeg')
    zy_path = os.path.join(pkg_path, 'face_img/zy.jpeg')
    mt_path = os.path.join(pkg_path, 'face_img/mt.jpeg')
    zcy_path = os.path.join(pkg_path, 'face_img/zcy.jpg')

    syl_image = face_recognition.load_image_file(syl_path)
    syl_face_encoding = face_recognition.face_encodings(syl_image)[0]

    zy_image = face_recognition.load_image_file(zy_path)
    zy_face_encoding = face_recognition.face_encodings(zy_image)[0]

    mt_image = face_recognition.load_image_file(mt_path)
    mt_face_encoding = face_recognition.face_encodings(mt_image)[0]

    zcy_image = face_recognition.load_image_file(zcy_path)
    zcy_face_encoding = face_recognition.face_encodings(zcy_image)[0]

    known_face_encodings = [
        syl_face_encoding,
        zy_face_encoding,
        mt_face_encoding,
        zcy_face_encoding
    ]
    known_face_names = [
        'Mike',
        'Jack',
        'Tom',
        'John'
    ]
    print(known_face_encodings)

    rospy.init_node('face_detect', anonymous=False)
    node = face_detect(known_face_encodings, known_face_names)
    rospy.spin()


