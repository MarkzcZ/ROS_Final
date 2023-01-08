# Mobile Robot Navigation and Control Final Lab Report

Chenzhi Zhan#12012505 , Yuxuan Zhang#12012508

Github repository link: [MarkzcZ/ROS_Final (github.com)](https://github.com/MarkzcZ/ROS_Final)



### Introduction

![image-20230108183954337](C:\Users\M__zzZ\AppData\Roaming\Typora\typora-user-images\image-20230108183954337.png)

The task of lab 7 starts from location P1, robot should move to P2,P3,P4 and back to P1 before moving toward the ArUco marker to recognize the ID of the marker placed near the red triangle. Upon recognizing the ID, the robot should beep with the buzzer on Turtlebot3 representing the ID, and then move to Pn before coming to a stop.

### Config

- Remote PC

  ```
  roscore
  ssh pi@192.168.3.33
  ```

  ```
  cd catkin_ws/src/
  ```

  

- SBC

  ```
  roslaunch bringup
  ```

  

​	