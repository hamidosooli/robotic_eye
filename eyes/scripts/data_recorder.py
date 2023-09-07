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
from decision_maker import PARTIAL_OCCLUSION_CONFIDENCE_BOUND, FULL_OCCLUSION_CONFIDENCE_BOUND, RESET_INTERVAL
import pandas as pd
from timeit import default_timer as timer
import os


MINIMUM_INTERVAL = 0.18

data = None


class DataRecorder:
    def __init__(self):


        self.state_x = message_filters.Subscriber("state_x", Float64, queue_size=1)
        self.state_y = message_filters.Subscriber("state_y", Float64, queue_size=1)

        self.leader = message_filters.Subscriber("leader", String, queue_size=1)


        #subcribes the the x and y control effort which is output from the pid package
        self.x_sub_left = message_filters.Subscriber("state_x_left", Float64, queue_size=1)
        self.y_sub_left = message_filters.Subscriber("state_y_left",Float64, queue_size=1)

        self.x_sub_right = message_filters.Subscriber("state_x_right", Float64, queue_size=1)
        self.y_sub_right = message_filters.Subscriber("state_y_right",Float64, queue_size=1)

        self.confidence_right = message_filters.Subscriber("confidence_right", Float64, queue_size=1)
        self.confidence_left = message_filters.Subscriber("confidence_left", Float64, queue_size=1)



        self.horizontal_error  = pd.DataFrame(columns=["Time", "Left", "Right", "Leader"])
        self.vertical_error  = pd.DataFrame(columns=["Time", "Left", "Right", "Leader"])


        self.partial_occlusion = pd.DataFrame(columns=["Time", "Left", "Right", "Leader"])
        self.full_occlusion = pd.DataFrame(columns=["Time", "Left", "Right", "Leader"])

        self.start = timer()
        self.last_timestamp_x = self.start
        self.last_timestamp_y = self.start
        self.last_timestamp_occlu = self.start

        self.horizontal_error_recorder = message_filters.ApproximateTimeSynchronizer([self.x_sub_left, self.x_sub_right, self.state_x], queue_size=1, slop=0.2, allow_headerless=True).registerCallback(self.horizontal_error_callback)
        self.vertical_error_recorder = message_filters.ApproximateTimeSynchronizer([self.y_sub_left, self.y_sub_right, self.state_y], queue_size=1, slop=0.2, allow_headerless=True).registerCallback(self.vertical_error_callback)

        self.occlusion_recorder = message_filters.ApproximateTimeSynchronizer([self.confidence_left, self.confidence_right, self.leader], queue_size=1, slop=0.2, allow_headerless=True).registerCallback(self.occlusion_callback)


    def horizontal_error_callback(self, left, right, leader):
       time = timer()
       if(time-self.last_timestamp_x>MINIMUM_INTERVAL):

            new_row = [time-self.start, left.data, right.data, leader.data]
            self.horizontal_error = pd.concat([self.horizontal_error, pd.DataFrame([new_row], columns=self.horizontal_error.columns)], ignore_index=True)
          
            self.last_timestamp_x = time

    def vertical_error_callback(self, left, right, leader):
       time = timer()
       if(time-self.last_timestamp_y>MINIMUM_INTERVAL):

            new_row = [time-self.start, left.data, right.data, leader.data]
            self.vertical_error = pd.concat([self.vertical_error, pd.DataFrame([new_row], columns=self.vertical_error.columns)], ignore_index=True)
          
            self.last_timestamp_y = time

    def occlusion_callback(self, confidence_left, confidence_right, leader):
        time = timer()
        if(time-self.last_timestamp_occlu>MINIMUM_INTERVAL):
                
                if(confidence_left.data<=FULL_OCCLUSION_CONFIDENCE_BOUND):
                    left_full_occlusion = 1
                else:
                    left_full_occlusion = 0

                if(confidence_right.data<=FULL_OCCLUSION_CONFIDENCE_BOUND):
                    right_full_occlusion = 1
                else:
                    right_full_occlusion = 0

                new_row = [time-self.start, left_full_occlusion, right_full_occlusion, leader.data]
                self.full_occlusion = pd.concat([self.full_occlusion, pd.DataFrame([new_row], columns=self.full_occlusion.columns)], ignore_index=True)
                

                if(confidence_left.data>FULL_OCCLUSION_CONFIDENCE_BOUND and confidence_left.data<=PARTIAL_OCCLUSION_CONFIDENCE_BOUND):
                    left_partial_occlusion = 1
                else:
                    left_partial_occlusion = 0

                if(confidence_right.data>FULL_OCCLUSION_CONFIDENCE_BOUND and confidence_right.data<=PARTIAL_OCCLUSION_CONFIDENCE_BOUND):
                    right_partial_occlusion = 1
                else:
                    right_partial_occlusion = 0

                new_row = [time-self.start, left_partial_occlusion, right_partial_occlusion, leader.data]
                self.partial_occlusion = pd.concat([self.partial_occlusion, pd.DataFrame([new_row], columns=self.partial_occlusion.columns)], ignore_index=True)
                
            
                self.last_timestamp_occlu = time


def transform_dataframe(df):
    result_df_left = pd.DataFrame(columns=['Time Range', 'Leader'])
    result_df_right = pd.DataFrame(columns=['Time Range', 'Leader'])

    current_range_start_left = None
    current_leader_left = None

    current_range_start_right = None
    current_leader_right = None

    for index, row in df.iterrows():
        print(row)
        time, left, right, leader = row

        if(left ==1 and right==1):
            leader += " (I)"

        if left == 1:
            if current_range_start_left  is None:
                current_range_start_left  = time
                current_leader_left  = leader
        else:
            if current_range_start_left  is not None:
                time_range = f"{current_range_start_left}-{time}"
                new_row = [time_range, current_leader_left]
                result_df_left = pd.concat([result_df_left, pd.DataFrame([new_row], columns=result_df_left.columns)], ignore_index=True)
                current_range_start_left  = None
                current_leader_left  = None

        if right == 1:
            if current_range_start_right is None:
                current_range_start_right = time
                current_leader_right = leader
        else:
            if current_range_start_right is not None:
                time_range = f"{current_range_start_right}-{time}"
                new_row = [time_range, current_leader_right]
                result_df_right = pd.concat([result_df_right, pd.DataFrame([new_row], columns=result_df_right.columns)], ignore_index=True)
                current_range_start_right = None
                current_leader_right = None

    return result_df_left, result_df_right

def shutdown_callback():

    path = "/home/nakulj/Desktop/PeARL2023/Robot/src/eyes/data/"+ str(RESET_INTERVAL)

    if not os.path.exists(path):
        os.makedirs(path)


    print("Saving data...")
    data.horizontal_error.to_excel(path + "/horizontal_error.xlsx", index=False)
    data.vertical_error.to_excel(path + "/vertical_error.xlsx", index=False)

    data.partial_occlusion.to_excel(path + "/partial_occlusion.xlsx", index=False)
    data.full_occlusion.to_excel(path+ "/full_occlusion.xlsx", index=False)

    left_partial, right_partial = transform_dataframe(data.partial_occlusion)
    left_full, right_full = transform_dataframe(data.full_occlusion)

    left_partial.to_excel(path + "/left_partial.xlsx", index=False)
    right_partial.to_excel(path+ "/right_partial.xlsx", index=False)

    left_full.to_excel(path+ "/left_full.xlsx", index=False)
    right_full.to_excel(path+ "/right_full.xlsx", index=False)
    print("Saved")

def main():
  
  #runs face detector node
  rospy.init_node('data_recorder')
  global data
  data = DataRecorder()

  rospy.on_shutdown(shutdown_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()





if __name__ == '__main__':
    main()
    