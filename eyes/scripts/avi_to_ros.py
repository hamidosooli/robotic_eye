#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_image(image_topic, avi_file_path):
    pub = rospy.Publisher(image_topic, Image, queue_size=1)
    bridge = CvBridge()

    # print("FFmpeg support:", cv2.getBuildInformation().count('WITH_FFMPEG'))
    # print(cv2.getBuildInformation())

    cap = cv2.VideoCapture(avi_file_path)

    codec = cv2.VideoWriter_fourcc('M', 'P', '4', 'V')

    cap.set(cv2.CAP_PROP_FOURCC, codec)

    rate = rospy.Rate(15)  

    ret, frame = cap.read()

    print(cap.isOpened())
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # print(ret)
        if ret:
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(image_msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        rospy.init_node('avi_to_image_topic_publisher', anonymous=True)
        image_topic = rospy.get_param('~image_topic')
        avi_file_path = rospy.get_param('~avi_file_path')
        publish_image(image_topic, avi_file_path)
    except rospy.ROSInterruptException:
        pass
