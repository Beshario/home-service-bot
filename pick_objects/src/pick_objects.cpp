#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double pickupgoal[2] = {4.125, 1.725};
double dropoffgoal[2] = {-1.59, -1.122};
int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
   //ready the publisher to broadcast 

  // Initialize the simple_navigation_goals node
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher status_pub = n.advertise<std_msgs::UInt8>("/robot_status", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = pickupgoal[0];
  pickup_goal.target_pose.pose.position.y = pickupgoal[1];
  pickup_goal.target_pose.pose.orientation.w = 2.00;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickup_goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
   ROS_INFO("Hooray, Robot made it to pick up location");
   std_msgs::UInt8 msg;
   msg.data = 1;
   status_pub.publish(msg);
   ROS_INFO("TRANSMITTED %d  over /robot_status", msg.data);
  } else { 
    ROS_INFO("The Robot hasnt made it for some reason :( :( ");
  }
  sleep(5);
    
      //move_base_msgs::MoveBaseGoal dropoff_goal;
 move_base_msgs::MoveBaseGoal dropoff_goal;

  // set up the frame parameters
  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  dropoff_goal.target_pose.pose.position.x = dropoffgoal[0];
  dropoff_goal.target_pose.pose.position.y = dropoffgoal[1];
  dropoff_goal.target_pose.pose.orientation.w = 1.00;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(dropoff_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, Robot made it to dropoff location");
    std_msgs::UInt8 msg3;
    msg3.data = 2;
    status_pub.publish(msg3);
    ROS_INFO("TRANSMITTED %d  over /robot_status", msg3.data);

    sleep(5);
  } else {
    ROS_INFO("The Robot hasnt made it for some reason :( :( ");
  }
  sleep(10);
  return 0;
}