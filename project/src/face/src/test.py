#!/usr/bin/python

import os
import face_recognition
import numpy as np
import cv2



if __name__ == '__main__':
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    pkg_path = os.path.dirname(dir_path)
    print(pkg_path)
    syl_path = os.path.join(pkg_path, 'face_img/syl.jpg')
    print(syl_path)
    # syl_image = face_recognition.load_image_file('/home/hrsd/humanoid_ws22-main-Project/Project_ws/src/face_recognition/face_img/syl.jpg')
    syl_image = face_recognition.load_image_file(syl_path)

    cv2.imshow('Video', syl_image)

# import sys
# import inspect
# import face_recognition

# try:
# 	print("face_recognition version:")
# 	print(face_recognition.__version__)
# except Exception as e:
# 	print(e)

# try:
# 	print()
# 	print("face_recognition path:")
# 	print(face_recognition.__file__)
# except Exception as e:
# 	print(e)


# try:
# 	print()
# 	print("face_recognition path (alternate method):")
# 	print(inspect.getfile(face_recognition))
# except Exception as e:
# 	print(e)

# print()
# print("Python version:")
# print(sys.version)
# print("Python executable:")
# print(sys.executable)

# try: 
# 	print("Python base path:")
# 	print(sys.base_prefix)
# except Exception as e:
# 	print(e)

# print("Python base path (exec):")
# print(sys.exec_prefix)
# print("Python system path:")
# print(sys.path)