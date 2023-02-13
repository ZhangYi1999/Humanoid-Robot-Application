Here is the readme from the project ticket train checker project made by Yi Zhang, Yinglei Song, Tao Ma, Chongyu Zhang. 

/*******************************************************************************************/
Dependencies
/*******************************************************************************************/
1.numpy
2.face_recognition

How to install numpy:
$ pip install numpy

How to install face_recognition:
$ pip install face-recognition

/*******************************************************************************************/
Packages needed for the whole task
/*******************************************************************************************/
Packages:

control
face

/*******************************************************************************************/
Network setup
/*******************************************************************************************/
You may need a separate laptop to run face recognition part.
It is important to setup your ROS IP and ROS Master.
For master(usually desktop machine), open .bashrc, and add:
export ROS_IP=localhost
export ROS_MASTER_URI=http://localhost:11311

For slave(usually laptop), open .bashrc, and add:
export ROS_IP=localhost
export ROS_MASTER_URI=http://IP_of_Master:11311

localhost is IP address of own machine.
For slave machine, you need to use the IP address of master in ROS_MASTER_URI.

Detailed reference: http://wiki.ros.org/ROS/NetworkSetup

/*******************************************************************************************/
How to run the code
/*******************************************************************************************/
On desktop machine
1. open one ternimal, run: $ cd project
2. run: $ catkin build
3. run: $ source devel/setup.bash
4. run: $ roslaunch control start.launch 

On laptop
1. open one ternimal, run: $ cd project
2. run: $ catkin build
3. run: $ source devel/setup.bash
4. run: $ rosrun face face_detect.py

