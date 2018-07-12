# !bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 4

#launch amcl with world made by slam previously
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 4

#Launch navigation and rviz navigation settings
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &

#pick objects node
xterm -e "rosrun pick_objects pick_objects_node" &

#start markers
xterm -e "rosrun add_markers basic_shapes" 

