#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
// char choose();

/** declare the coordinates of interest **/
double point1x = -0.275;
double point1y = 0.7191;
double point2x = 2.5104 ;
double point2y = -2.1866;
double point3x = -3.227 ;
double point3y = -1.994;
double point4x = -0.5420 ;
double point4y = -4.9148;

bool goalReached = false;

 int main(int argc, char** argv){
   ros::init(argc, argv, "simple_navigation_goals");
   ros::NodeHandle n;
   ros::spinOnce();
   

   
  int choice=0;
   do{
      if (choice == 0){
         goalReached = moveToGoal(point1x, point1y);
      }else if (choice == 1){
         goalReached = moveToGoal(point2x, point2y);
      }else if (choice == 2){
         goalReached = moveToGoal(point3x, point3y);
      }else if (choice == 3){
         goalReached = moveToGoal(point4x, point4y);
      }
      
         if (goalReached){
            ROS_INFO("Congratulations!");
            choice++;
            
            // ros::spinOnce();
            // ros::spinOnce();
         }else{
            ROS_INFO("Hard Luck!");
         }
      
   }while(choice < 4);

   return 0;
}


bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }

}



// int main(int argc, char** argv){
//  ros::init(argc, argv, "simple_navigation_goals");

// //tell the action client that we want to spin a thread by default
// MoveBaseClient ac("move_base", true);

// //wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//   	ROS_INFO("Waiting for the move_base action server to come up");
//   }
   
//   move_base_msgs::MoveBaseGoal goal;
  
//   //we'll send a goal to the robot to move 1 meter forward
//   goal.target_pose.header.frame_id = "base_link";
//   goal.target_pose.header.stamp = ros::Time::now();
  
//   goal.target_pose.pose.position.x = 1.0;
//   goal.target_pose.pose.orientation.w = 1.0;
  
//   ROS_INFO("Sending goal");
//   ac.sendGoal(goal);
  
//   ac.waitForResult();
  
//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     ROS_INFO("Hooray, the base moved 1 meter forward");
//      else
//       ROS_INFO("The base failed to move forward 1 meter for some reason");
  
//   return 0;
// }
