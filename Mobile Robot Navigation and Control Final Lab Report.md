# Final Lab Report

Chenzhi Zhan#12012505 , Yuxuan Zhang#12012508

Github repository link: [MarkzcZ/ROS_Final (github.com)](https://github.com/MarkzcZ/ROS_Final)



### Introduction

![image-20230111203753061](C:\Users\Mark Z\AppData\Roaming\Typora\typora-user-images\image-20230111203753061.png)

The task of lab 7 starts from location P1, robot should move to P2,P3,P4 and back to P1 before moving toward the ArUco marker to recognize the ID of the marker placed near the red triangle. Upon recognizing the ID, the robot should beep with the buzzer on Turtlebot3 representing the ID, and then move to Pn before coming to a stop.

### Config

- Remote PC

  ```
  roscore
  ```
  ```
  ssh pi@192.168.3.33
  ```
  ```
  cd catkin_ws/src/simple_navigation_goals/src/
  
  python map_navigation.py
  ```

  ```
  roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/classroom.yaml
  ```

  

- SBC

  ```
  roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```

  

​	
