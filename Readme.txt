Here is the readme from the project ticket train checker project made by Yi Zhang, Tao Ma, Yinglei Song, Chongyu Zhang. 

/*******************************************************************************************/
Important libraries (dependencies) which need to be installed
/*******************************************************************************************/
1.dlib
2.face_recognition


How to install dlib:
$ git clone https://github.com/davisking/dlib.git
$ cd dlib
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build
$ cd ..
$ python2(3) setup.py install

How to install face_recognition:
$ pip2(3) install face-recognition

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
For slave machine, you need to replace localhost to the IP address of master.

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

