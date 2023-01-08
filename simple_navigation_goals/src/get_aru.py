import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point,Twist
import message_filters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import numpy as np

destination=-1



class aru_recog():
    def __init__(self):
        rospy.init_node('aru_recog',anonymous=True)
        self.bridge = CvBridge()
        self.cmd_vel=rospy.Subscriber("raspicam_node/image_raw",Image,self.aru_callback)
        rospy.spin()

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
        print(destination)
      cv2.imshow("frame", cv_img)
      cv2.waitKey(1)

    def getDes(self):
      return destination
      



class aru_park():
    def __init__(self):
        rospy.init_node('aru_park',anonymous=True)
        self.bridge = CvBridge()
        # sub_info_aru=rospy.Subscriber("/aruco_single/pose",)
        # aru_listener=message_filters.TimeSynchronizer(sub_info_aru,10)
        # aru_listener.registerCallback(aru_callback)
        self.cmd_vel=rospy.Subscriber("raspicam_node/image_raw",Image,self.aru_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.velocity=Twist()
        rospy.spin()

    def aru_callback(self,data_array):
      cv_img = self.bridge.imgmsg_to_cv2(data_array,"bgr8")
      
      gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
      aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
      parameters = aruco.DetectorParameters_create()
      corners,ids,rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
      aruco.drawDetectedMarkers(cv_img,corners,ids)

      if not (ids is None):
        print(ids[0][0])
        
      else:
        print("fail")
      cv2.imshow("frame", cv_img)
      cv2.waitKey(1)


      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      # #wait for the action server to come up
      # while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      #   rospy.loginfo("Waiting for the move_base action server to come up")
      goal = MoveBaseGoal()

    #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # if x_distance>0.03:
      #   self.velocity.linear.x = 0.04
      #   self.velocity.angular.z = -0.3
      # elif x_distance<-0.03:
      #   self.velocity.linear.x = 0.04
      #   self.velocity.angular.z = 0.3
      # elif z_distance>0.3:
      #   self.velocity.linear.x = 0.04
      #   self.velocity.angular.z = 0
      # elif z_distance<=0.1 and z_distance>0.05:
      #   self.velocity.linear.x = 0
      #   self.velocity.angular.z = 0
      # else:
      #   self.velocity.linear.x = 0
      #   self.velocity.angular.z = 0

      # rospy.loginfo(self.velocity)
      self.pub.publish(self.velocity)

 
if __name__ == '__main__':
    try:
        aru_recog()
    except:
        rospy.loginfo("GoForward node terminated.")
