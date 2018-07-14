# Home-Service-Robot
A robot that utilizes SLAM, localization and autonomous path planning to pick a goal, and deliver it to a drop off location:
if you have a catkin_ws folder then skip the follwoing step.
however if you do not have an active ROS workspace, you can create one by:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Clone the required repository to the `~/catkin_ws/src` folder. Note that this repository already includes official ROS packages compatible with this repository:gmapping, turtlebot_teleop, turtlebot_rviz_launchers, and turtlebot_gazebo. 

Make sure you have all the dependencies:

<p align="center"><img src="./20180711_072600.gif"></p>

```sh
cd ~/catkin_ws/src
git clone https://github.com/Beshario/udacity-final-home-service-bot
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
cd ~/catkin_ws
catkin_make
```

To run the home service robot script files, you will need to source the ROS environment variabes in a terminal by:
```sh
cd ~/catkin_ws
source devel/setup.bash
./src/home_service.sh
```
you whichever script you like to run the project on different modes:
If everything is ready to rock'n'roll, the below is the result of running ./home_service.sh commands (at 5x speed).



