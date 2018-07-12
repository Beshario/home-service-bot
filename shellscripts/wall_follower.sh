# !bin/sh
#for sourcing

#Launching turtle bot in U world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 4

#run slam
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" & 
#custom_gmapping_launch_file:=/home/workspace/catkin_ws/src/path_planning_project/turtlebot_navigation/uworld_gmapping.launch.xml
sleep 4

#run navigation
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3

# run wall follower
xterm -e "rosrun wall_follower wall_follower_node"
