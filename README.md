# lidar_tracking

If you don't have ros' catkin_ws, Please follow this page : [http://wiki.ros.org/ko/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/ko/catkin/Tutorials/create_a_workspace)

#### First terminal
'''
$ roscore
'''

#### Second terminal
'''
$ cd ~/catkin_ws/src/

$ git clone https://github.com/kimhongsuk/lidar_tracking.git

$ cd ..

$ catkin_make

$ sudo ldconfig

$ source devel/setup.bash

$ rosrun lidar_tracking lidar_tracking_node
'''

#### Third terminal
'''
$ rosbag play {Your rosbag file directory}
'''
