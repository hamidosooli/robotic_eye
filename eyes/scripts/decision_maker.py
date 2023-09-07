#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import time
import scipy
import math
from math import pi, sqrt, sin, cos, atan2


PARTIAL_OCCLUSION_CONFIDENCE_BOUND = 0.8
FULL_OCCLUSION_CONFIDENCE_BOUND = 0
RESET_INTERVAL = 5


class DecisionMaking:
    def __init__(self):

        self.BI = np.zeros((1, 2))

        self.reset()
        self.n = 0

        self.state_x = rospy.Publisher("state_x",Float64,queue_size=1)
        self.state_y = rospy.Publisher("state_y",Float64,queue_size=1)
        self.leader = rospy.Publisher("leader",String,queue_size=1)

        #subcribes the the x and y control effort which is output from the pid package
        self.x_sub_left = message_filters.Subscriber("state_x_left", Float64, queue_size=1)
        self.y_sub_left = message_filters.Subscriber("state_y_left",Float64, queue_size=1)

        self.x_sub_right = message_filters.Subscriber("state_x_right", Float64, queue_size=1)
        self.y_sub_right = message_filters.Subscriber("state_y_right",Float64, queue_size=1)

        self.confidence_right = message_filters.Subscriber("confidence_right", Float64, queue_size=1)
        self.confidence_left = message_filters.Subscriber("confidence_left", Float64, queue_size=1)

        #synchronizes x and y control effort
        self.ts = message_filters.ApproximateTimeSynchronizer([self.x_sub_left, self.y_sub_left, self.x_sub_right, self.y_sub_right, self.confidence_left, self.confidence_right], queue_size=1, slop=1, allow_headerless=True).registerCallback(self.decision)

    def reset(self):
        self.LeftPlayer = 1000 * np.ones((4, 4))
        self.RightPlayer = 1000 * np.ones((4, 4))
        self.L_Theta = np.zeros((4, 4))
        self.R_Theta = np.zeros((4, 4))

    def BackwardInduction(self, matrix1, matrix2, whois):
        i = 1
        j = 1
        counter = 0
        Nextindex1 = np.zeros((8, 2))
        Nextindex2 = np.zeros((6, 2))
        if whois == 1:
            Player2matrix = matrix2.copy()
            Player1matrix = matrix1.copy()  # leader
        elif whois == 2:
            Player2matrix = matrix1.copy()
            Player1matrix = matrix2.copy()  # leader
    #########################################################
        # First Quarter
        if Player2matrix[i, j] < Player2matrix[i, j - 1]:
            Nextindex1[counter, :] = [i, j]
        else:
            Nextindex1[counter, :] = [i, j - 1]

        if Player2matrix[i - 1, j] < Player2matrix[i - 1, j - 1]:
            Nextindex2[counter, :] = [i - 1, j]
        else:
            Nextindex2[counter, :] = [i - 1, j - 1]

        counter += 1
    #########################################################
        # Second Quarter
        if Player2matrix[i, j + 2] < Player2matrix[i, j + 1]:
            Nextindex1[counter, :] = [i, j + 2]
        else:
            Nextindex1[counter, :] = [i, j + 1]

        if Player2matrix[i - 1, j + 2] < Player2matrix[i - 1, j + 1]:
            Nextindex2[counter, :] = [i - 1, j + 2]
        else:
            Nextindex2[counter, :] = [i - 1, j + 1]

        counter += 1
    #########################################################
        # Third Quarter
        if Player2matrix[i + 2, j] < Player2matrix[i + 2, j - 1]:
            Nextindex1[counter, :] = [i + 2, j]
        else:
            Nextindex1[counter, :] = [i + 2, j - 1]

        if Player2matrix[i + 1, j] < Player2matrix[i + 1, j - 1]:
            Nextindex2[counter, :] = [i + 1, j]
        else:
            Nextindex2[counter, :] = [i + 1, j - 1]

        counter += 1
    #########################################################
        # Fourth Quarter
        if Player2matrix[i + 2, j + 2] < Player2matrix[i + 2, j + 1]:
            Nextindex1[counter, :] = [i + 2, j + 2]
        else:
            Nextindex1[counter, :] = [i + 2, j + 1]

        if Player2matrix[i + 1, j + 2] < Player2matrix[i + 1, j + 1]:
            Nextindex2[counter, :] = [i + 1, j + 2]
        else:
            Nextindex2[counter, :] = [i + 1, j + 1]

        counter += 1
    #########################################################
        # Second Step
        for k in range(0, 3, 2):
            l1 = Nextindex1[k, 0]
            r1 = Nextindex1[k, 1]

            l2 = Nextindex2[k, 0]
            r2 = Nextindex2[k, 1]

            if Player1matrix[int(l1), int(r1)] < Player1matrix[int(l2), int(r2)]:
                Nextindex1[counter, :] = Nextindex1[k, :]
            else:
                Nextindex1[counter, :] = Nextindex2[k, :]
    #########################################################
            l1 = Nextindex1[k + 1, 0]
            r1 = Nextindex1[k + 1, 1]

            l2 = Nextindex2[k + 1, 0]
            r2 = Nextindex2[k + 1, 1]

            if Player1matrix[int(l1), int(r1)] < Player1matrix[int(l2), int(r2)]:
                Nextindex2[counter, :] = Nextindex1[k + 1, :]
            else:
                Nextindex2[counter, :] = Nextindex2[k + 1, :]
            counter += 1
    #########################################################
        # Third Step
        for n in range(4, 6):
            l1 = Nextindex1[n, 0]
            r1 = Nextindex1[n, 1]

            l2 = Nextindex2[n, 0]
            r2 = Nextindex2[n, 1]

            if Player2matrix[int(l1), int(r1)] < Player2matrix[int(l2), int(r2)]:
                Nextindex1[counter, :] = Nextindex1[n, :]
            else:
                Nextindex1[counter, :] = Nextindex2[n, :]
            counter += 1
    #########################################################
        # Fourth Step
        l1 = Nextindex1[6, 0]
        r1 = Nextindex1[6, 1]

        l2 = Nextindex1[7, 0]
        r2 = Nextindex1[7, 1]

        if Player1matrix[int(l1), int(r1)] < Player1matrix[int(l2), int(r2)]:
            self.BI = Nextindex1[6, :]
        else:
            self.BI = Nextindex1[7, :]

    def decision(self, x_left, y_left, x_right, y_right, confidence_left, confidence_right):

        



        # print("made it")


        # print(str(confidence_left) + " " + str(confidence_right))

        if (confidence_right.data<=PARTIAL_OCCLUSION_CONFIDENCE_BOUND and confidence_left.data>PARTIAL_OCCLUSION_CONFIDENCE_BOUND) or (confidence_right.data<=FULL_OCCLUSION_CONFIDENCE_BOUND and confidence_left.data>FULL_OCCLUSION_CONFIDENCE_BOUND):
            # Left Player  is optimal leader due to right partial or full occlusion
            leader_msg = String()
            leader_msg.data = "left"

            msg_state_x = Float64()
            msg_state_x.data = x_left.data

            msg_state_y = Float64()
            msg_state_y.data = y_left.data

        elif (confidence_left.data<=PARTIAL_OCCLUSION_CONFIDENCE_BOUND and confidence_right.data>PARTIAL_OCCLUSION_CONFIDENCE_BOUND) or (confidence_left.data<=FULL_OCCLUSION_CONFIDENCE_BOUND and confidence_right.data>FULL_OCCLUSION_CONFIDENCE_BOUND):
             # Right Player  is optimal leader due to left partial or full occlusion
            leader_msg = String()
            leader_msg.data = "right"

            msg_state_x = Float64()
            msg_state_x.data = x_right.data

            msg_state_y = Float64()
            msg_state_y.data = y_right.data
        else:

            

            if self.n > RESET_INTERVAL:
                self.n = 0
                self.reset()
            else:
                self.n += 1

            x_err_left = x_left.data 
            y_err_left = y_left.data 
            
            x_err_right = x_right.data 
            y_err_right = y_right.data 

            # Left (Row Player) strategies
            if x_err_left < 0 and y_err_left < 0: # L U (Picture 1'st Quarter)
                row = 0
            elif x_err_left < 0 and y_err_left > 0: # L D (Picture 4'th Quarter)
                row = 1
            elif x_err_left > 0 and y_err_left < 0: # R U (Picture 2'nd Quarter)
                row = 2
            else:# R D (Picture 3'rd Quarter)
                row = 3            


            # Right (Column Player) strategies
            if x_err_right < 0 and y_err_right < 0: # L U
                col = 0
            elif x_err_right < 0 and y_err_right > 0: # L D
                col = 1
            elif x_err_right > 0 and y_err_right < 0: # R U
                col = 2
            else: # R D
                col = 3

            # print("row: " + str(row) + " col: " + str(col))

            # print(str(x_err_left) + " " + str(y_err_left) + " " +str(x_err_right) + " " + str(y_err_right) + " ")

            

            self.LeftPlayer[row, col] = sqrt(x_err_left**2 + y_err_left**2)
            self.RightPlayer[row, col] = sqrt(x_err_right**2 + y_err_right**2)

            self.L_Theta[row, col] = atan2(y_err_left,x_err_left)
            self.R_Theta[row, col] = atan2(y_err_right,x_err_right)


            # Leaf Algorithm
            self.BackwardInduction(self.LeftPlayer, self.RightPlayer, 1)
            l1 = self.BI[0]
            r1 = self.BI[1]
            self.BackwardInduction(self.LeftPlayer, self.RightPlayer, 2)
            l2 = self.BI[0]
            r2 = self.BI[1]


            # print(str(l1) + " " + str(r1) + " " + str(l2) + " " + str(r2) + " ")
            # print(str(sqrt( self.LeftPlayer[int(l1), int(r1)]**2 + self.RightPlayer[int(l1), int(r1)]**2 )) + " " + str(sqrt( self.LeftPlayer[int(l2), int(r2)]**2 + self.RightPlayer[int(l2), int(r2)]**2 )))

            if (sqrt( self.LeftPlayer[int(l1), int(r1)]**2 + self.RightPlayer[int(l1), int(r1)]**2 ) <=
                sqrt( self.LeftPlayer[int(l2), int(r2)]**2 + self.RightPlayer[int(l2), int(r2)]**2 )):

                # Left Player  is optimal leader
                leader_msg = String()
                leader_msg.data = "left"

                msg_state_x = Float64()
                msg_state_x.data = self.LeftPlayer[int(l1), int(r1)] * cos(self.L_Theta[int(l1), int(r1)])

                msg_state_y = Float64()
                msg_state_y.data = self.LeftPlayer[int(l1), int(r1)] * sin(self.L_Theta[int(l1), int(r1)]) 
            else:
                # Right Player  is optimal leader
                leader_msg = String()
                leader_msg.data = "right"

                msg_state_x = Float64()
                msg_state_x.data = self.RightPlayer[int(l2), int(r2)] * cos(self.R_Theta[int(l2), int(r2)])

                msg_state_y = Float64()
                msg_state_y.data = self.RightPlayer[int(l2), int(r2)] * sin(self.R_Theta[int(l2), int(r2)]) 


            # print(str(msg_state_x) + " " + str(msg_state_y) +  " " + str(x_err_left) + " " + str(y_err_left))

        try:
            self.state_x.publish(msg_state_x)
            self.state_y.publish(msg_state_y)
            self.leader.publish(leader_msg)
        except Exception as e:
                print(e)




def main():
  
  #runs face detector node
  rospy.init_node('decision_maker', anonymous=True)
  decisionMaker = DecisionMaking()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
    