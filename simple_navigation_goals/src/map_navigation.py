import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point,Twist
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
from turtlebot3_msgs.msg import *


destination = -1
count = 0


class aru_recog():
    def __init__(self):
        # rospy.init_node('aru_recog',anonymous=True)
        self.bridge = CvBridge()
        self.cmd_vel=rospy.Subscriber("raspicam_node/image_raw",Image,self.aru_callback)


    def aru_callback(self,data_array):
      cv_img = self.bridge.imgmsg_to_cv2(data_array,"bgr8")
      gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
      aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
      parameters = aruco.DetectorParameters_create()
      corners,ids,rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
      aruco.drawDetectedMarkers(cv_img,corners,ids)
      global destination
      if not (ids is None):
        destination=int(ids[0][0]) 
      cv2.imshow("frame", cv_img)
      cv2.waitKey(1)





class map_navigation():
  def choose(self):
    choice='q'

    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|PRESSE A KEY:")
    rospy.loginfo("|'0': Cafe ")
    rospy.loginfo("|'1': Office 1 ")
    rospy.loginfo("|'2': Office 2 ")
    rospy.loginfo("|'3': Office 3 ")
    rospy.loginfo("|'q': Quit ")
    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|WHERE TO GO?")
    choice = input()
    return choice


  def __init__(self):
    # declare the coordinates of interest
    self.xCafe = -0.287
    self.yCafe = 0.8043
    self.xOffice1 = 2.6204
    self.yOffice1 = -2.2366
    self.xOffice2 = -3.28
    self.yOffice2 = -1.86
    self.xOffice3 = -0.5520
    self.yOffice3 = -4.9448

    
    self.have_travel=0

    self.goalReached = False
    # initiliaze
    self.pub = rospy.Publisher("/sound", Sound, queue_size=10)
    self.msg = Sound()
    
    choice = self.choose()

    global count
    while choice != 'q':
      if (choice == 1):
        self.goalReached = self.moveToGoal(self.xCafe, self.yCafe)

      elif (choice == 0):
        self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)
        self.have_travel=1

      elif (choice == 2):
        self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)

      elif (choice == 3):
        self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)
      
      if (choice!='q'):
        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          if destination!=-1 and self.have_travel==1:
            choice=destination-1
            if(count == 0):
              self.msg.value = destination - 1
              self.pub.publish(self.msg)
              count = count + 1
          else:
            choice = self.choose()
        else:
          rospy.loginfo("Hard Luck!")
          choice = self.choose()
    


  def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

  def moveToGoal(self,xGoal,yGoal):

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)



      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")
      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/

      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.945534610194 #0
      goal.target_pose.pose.orientation.w = 0.325521582888 #1

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)

      ac.wait_for_result(rospy.Duration(60))

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              return True

      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False


if __name__ == '__main__':
    try:
      rospy.init_node('map_navigation', anonymous=False)
      aru_recog()
      map_navigation()
      rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")