#!/usr/bin/env python3

import rospy
from display import Display
import cv2

def main():
  
  #ros node for publishing the left camera display
  rospy.init_node('right_display', anonymous=True)

  #adjust the webcam index and the path to the calibration data accordingly
  display = Display(2, "/camera/right", "/home/nakulj/Desktop/PeARL2023/Robot/src/eyes/calibrationdata/right.yaml")

  #publishes webcam feed
  while(1):
    display.publish()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()