from sensor_msgs.msg import Image
import rospy
import actionlib
import cv2
from cv2 import aruco
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point,Twist
from cv_bridge import CvBridge, CvBridgeError
from turtlebot3_msgs.msg import *

class Aru(smach.State):
    def __init__(self):
        # rospy.init_node('aru_recog',anonymous=True)
        smach.State.__init__(self, outcomes=['outcome1'])

        # self.bridge = CvBridge()
        # self.pub = rospy.Publisher("/sound", Sound, queue_size=10)
        # self.msg = Sound()
        # self.cmd_vel=rospy.Subscriber("raspicam_node/image_raw",Image,self.aru_callback)
        
    def executed(self):
        rospy.loginfo('Executing state Aru')
        self.bridge = CvBridge()
        # self.pub = rospy.Publisher("/sound", Sound, queue_size=10)
        # self.msg = Sound()
        self.cmd_vel=rospy.Subscriber("raspicam_node/image_raw",Image,self.aru_callback)
        if destination != -1:
            return 'outcome1'


    def talker(self):
      self.msg.value = 2
      # rate = rospy.Rate(1)
      self.pub.publish(self.msg)
      # rate.sleep()


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
        
        # self.talker()
      # else:
      #   self.msg.value = -1
      #   self.pub.publish(self.msg)
      


      cv2.imshow("frame", cv_img)
      cv2.waitKey(1)

class Map(smach.State):

  def __init__(self):
    smach.State.__init__(self, outcomes=['outcome2'])


  def execute(self):
    rospy.loginfo('Executing state Map')
    self.INI(self)
    return 'outcome2'
    
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


  def INI(self):
    # declare the coordinates of interest
    self.xCafe = -0.287
    self.yCafe = 0.8043
    self.xOffice1 = 2.6204
    self.yOffice1 = -2.2366
    self.xOffice2 = -3.28
    self.yOffice2 = -1.86
    self.xOffice3 = -0.5520
    self.yOffice3 = -4.9448

    self.parkpointx=1.835
    self.parkpointy=-1.866
    
    have_travel=0

    self.goalReached = False
    # initiliaze
    
    
    choice = self.choose()

    # if (choice == 0):
    #   self.goalReached = self.moveToGoal(self.xCafe, self.yCafe)

    # elif (choice == 1):

    #   self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)

    # elif (choice == 2):

    #   self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)

    # elif (choice == 3):

    #   self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)

    # elif (choice==4):
    #   self.goalReached = self.moveToGoal(self.parkpointx, self.parkpointy)

    
    # if (choice!='q'):
    #   if (self.goalReached):
    #     rospy.loginfo("Congratulations!")
    #     rospy.loginfo(destination)
    #   else:
    #     rospy.loginfo("Hard Luck!")


    while choice != 'q':
      if (choice == 1):
        self.goalReached = self.moveToGoal(self.xCafe, self.yCafe)

      elif (choice == 0):
        self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)

      elif (choice == 2):
        self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)

      elif (choice == 3):
        self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)
        have_travel=1

      elif (choice==4):
        # near park
        self.goalReached = self.moveToGoal(self.parkpointx, self.parkpointy)
        while(destination == -1):
          continue
      if (choice!='q'):
        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          if destination!=-1 and have_travel==1:
            choice=destination-1
          else:
            choice = self.choose()
     
          # rospy.spin()
          # rospy.spin()
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

def main():
  rospy.init_node('smach_example_state_machine')
# Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
  with sm:
    smach.StateMachine.add('Aru', Aru(), 
                                  transitions={'outcome1':'Map', 
                                               })
    smach.StateMachine.add('Map', Map(), 
                                  transitions={'outcome2':'outcome'})
   
       # Execute SMACH plan
  outcome = sm.execute()

if __name__ == '__main__':
    main()