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
from messages.msg import Point, PointList
import math


class Detector:

    def __init__(self):

        #model used to detect faces
        self.model_path = '/home/nakulj/Desktop/PeARL2023/Robot/src/eyes/weights/yolov5n-face.onnx'
        self.net = self.build_model(False)


        # publishes the x and y states and setpoints for the pid packages to output an x and y control effort
        self.state_x_pub = rospy.Publisher("state_x",Float64,queue_size=1)
        self.state_y_pub = rospy.Publisher("state_y",Float64,queue_size=1)
        self.setpoint_x = rospy.Publisher("setpoint_x",Float64,queue_size=1)
        self.setpoint_y = rospy.Publisher("setpoint_y",Float64,queue_size=1)


        #subcriber for the image publisher
        #right now it only uses the left image to detect faces
        self.bridge = CvBridge()
        self.limage_sub = message_filters.Subscriber("/camera/left/image_raw",Image, queue_size=1, buff_size=2**24).registerCallback(self.callback)
        
    
        

        #dimensions of input image for the yolo model
        self.INPUT_HEIGHT = 320
        self.INPUT_WIDTH = 320
    


        #subscribes to and stores depth of the face
        self.depth_sub = message_filters.Subscriber("face_depth",Float64, queue_size=1).registerCallback(self.depthCallback)
        self.depth = 0
        

    #callback to store the face depth estimate
    def depthCallback(self, data):
        if not math.isnan(data.data):
            self.depth = data.data

    #callback for the left camera image 
    #does all the face detection stuff
    def callback(self,left):

        #gets the image
        try:
            cv_image_left = CvBridge().imgmsg_to_cv2(left)
        except CvBridgeError as e:
            print(e)


        #formats and detects faces
        input = self.format_yolov5(cv_image_left)
        output = self.detect(input, self.net)
        class_ids, confidences, boxes = self.wrap_detection(input, output[0])


        #labels each face with a green rectangle and the depth  
        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = (0, 255, 0)
            cv2.rectangle(cv_image_left, box, color, 2)
            cv2.rectangle(cv_image_left, (box[0], box[1]), (box[0] + box[2], box[1]+box[3]), color)
            cv2.putText(cv_image_left, "", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,0))

        #shows image
        cv2.imshow("images", cv_image_left)
        cv2.waitKey(1)


        #takes the first detected face and publishes x and y values to state topic
        if len(boxes) != 0:
            #takes the first face
            point = boxes[0]

            #creates x and y state messages
            msg_state_x = Float64()
            msg_state_x.data = point[0] + (point[2]/2)

            msg_state_y = Float64()
            msg_state_y.data = point[1] + (point[3]/2)

            #creates the x and y setpoint messages which will be the center of the frame
            msg_setpoint_x = Float64()
            msg_setpoint_x.data = 240

            msg_setpoint_y = Float64()
            msg_setpoint_y.data = 240

            try:
                #publishes states and setpoints
                self.state_x_pub.publish(msg_state_x)
                self.state_y_pub.publish(msg_state_y)
                self.setpoint_x.publish(msg_setpoint_x)
                self.setpoint_y.publish(msg_setpoint_y)
            except Exception as e:
                print(e)



    #makes the frame the proper dimensions and format for the yolo model
    def format_yolov5(self,frame):
        frame = ((frame / np.max(frame)) * 255).astype('uint8')
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        result.shape
        return result



    #detects faces in the frame using the yolo model
    def detect(self,image, net):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.INPUT_WIDTH, self.INPUT_HEIGHT), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds


    #converts face detection results from yolo output to class, confidence, box
    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]
        

        #conversion from yolo output image back into 640, 480 frame
        image_width, image_height, _ = input_image.shape

        x_factor = image_width / self.INPUT_WIDTH
        y_factor =  image_height / self.INPUT_HEIGHT


        #loop through each output
        for r in range(rows):
            row = output_data[r]

            confidence = row[4]

            #checks for at least 0.4 confidence on face detection
            if confidence >= 0.4:

                
                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    #gets the x and y of the top left corner as well as image height and width
                    #makes boxes lists
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        #gets the indexes of each box
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        #adds class id, confidence, and box coordinates to each index spot
        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes



    #retrieves model from given path 
    def build_model(self,is_cuda):
        # net = cv2.dnn.readNet('face_detection_yolov5s.onnx')
        net = cv2.dnn.readNet(self.model_path)
        if is_cuda:
            print("Attempting to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            
        return net
    


def main():
  
  #runs face detector node
  rospy.init_node('face_detector', anonymous=True)
  detector = Detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()