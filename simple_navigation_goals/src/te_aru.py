import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point,Twist
import message_filters
from geometry_msgs.msg import PoseStamped



# def aru_listener():
#   rospy.init_node('listenner_aru',anonymous=False)
#   # sub_info_aru=rospy.Subscriber("/aruco_single/pose",)
#   # aru_listener=message_filters.TimeSynchronizer(sub_info_aru,10)
#   # aru_listener.registerCallback(aru_callback)
#   rospy.Subscriber("aruco_single/pose",PoseStamped,aru_callback)
#   pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,queue_size=10)
#   rospy.spin()
class aru_park():
    def __init__(self):
        rospy.init_node('aru_park',anonymous=True)
        # sub_info_aru=rospy.Subscriber("/aruco_single/pose",)
        # aru_listener=message_filters.TimeSynchronizer(sub_info_aru,10)
        # aru_listener.registerCallback(aru_callback)
        self.cmd_vel=rospy.Subscriber("aruco_single/pose",PoseStamped,self.aru_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        self.velocity=Twist()
        rospy.spin()

    def aru_callback(self,data_array):
      x_distance=data_array.pose.position.x
      z_distance=data_array.pose.position.z
      rospy.loginfo(x_distance)

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      # ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      # #wait for the action server to come up
      # while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      #   rospy.loginfo("Waiting for the move_base action server to come up")
      goal = MoveBaseGoal()

    #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      if x_distance>0.03:
        self.velocity.linear.x = 0.04
        self.velocity.angular.z = -0.3
      elif x_distance<-0.03:
        self.velocity.linear.x = 0.04
        self.velocity.angular.z = 0.3
      elif z_distance>0.3:
        self.velocity.linear.x = 0.04
        self.velocity.angular.z = 0
      elif z_distance<=0.1 and z_distance>0.05:
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
      else:
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0

      rospy.loginfo(self.velocity)
      self.pub.publish(self.velocity)

 
if __name__ == '__main__':
    try:
        aru_park()
    except:
        rospy.loginfo("GoForward node terminated.")