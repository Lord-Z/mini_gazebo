# mini_gazebo

Tianbot_mini simulation.Gazebo simulation of two wheel differential car, including map building and navigation
## 这是一个tianbot_mini的gazebo仿真,Ubuntu18.04+ROS melodic环境，使用教程如下

### 1.功能包使用
新建工作空间

将mini_sim/racetrack_V2移至.gazebo/models目录下面

安装一些功能包

pip install playsound

sudo apt-get install ros-melodic-amcl

sudo apt-get install ros-melodic-navigation

sudo apt-get install ros-melodic-joint-state-controller

sudo apt-get install ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-controller-manager

将mini_sim放进工作空间目录下面进行编译

### 2.小车建图
roslaunch mini_gazebo simulation_camera.launch

roslaunch mini_nav gmapping_demo.launch

roslaunch mini_nav mini_teleop.launch

### 3.小车导航
roslaunch mini_gazebo simulation_camera.launch

roslaunch mini_nav nav_demo.launch

用红色箭头nav_goal点击rviz地图中位置即可让小车前往目标点

比赛二维码导航demo

cd xx_ws/src/mini_sim/mini_nav/scripts/

./run3.py


