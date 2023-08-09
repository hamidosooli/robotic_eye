#!/usr/bin/env python3

from robot import Robot
import rospy
import message_filters
from std_msgs.msg import Float64

#node that moves the robot to center the face
class Mover:
   
   def __init__(self):
      #creates robot object for all the hardware writes
      self.robot = Robot()
      
      #subcribes the the x and y control effort which is output from the pid package
      self.x_correction_sub = message_filters.Subscriber("control_effort_x", Float64, queue_size=1)
      self.y_correction_sub = message_filters.Subscriber("control_effort_y",Float64, queue_size=1)

      #synchronizes x and y control effort
      self.ts = message_filters.ApproximateTimeSynchronizer([self.x_correction_sub, self.y_correction_sub], queue_size=1, slop=0.1, allow_headerless=True).registerCallback(self.callback)

      
    
  #callbrack for sync x and y control effort
   def callback(self, x, y):
      
      #increments servos by the x and y control effort
      x_move = -x.data
      y_move = -y.data
      self.robot.incrementPositions([x_move, y_move, -x_move, y_move])





def main():
  rospy.init_node('eye_mover', anonymous=True)
  mover = Mover()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
