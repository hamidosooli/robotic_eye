#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

class Depth:
   
   def __init__(self):
      #publishes depth of face in metres
      self.face_depth_pub = rospy.Publisher("face_depth",Float64,queue_size=1)


      #subscribes to the x and y coordinates of the face and the point cloud
      self.x_state_sub = message_filters.Subscriber("/state_x", Float64, queue_size=1).registerCallback(self.callback_x)
      self.y_state_sub = message_filters.Subscriber("/state_y",Float64, queue_size=1).registerCallback(self.callback_y)
      self.pcl_sub = message_filters.Subscriber("/camera/points2",PointCloud2, queue_size=1, buff_size=2**24)
      self.pcl_sub.registerCallback(self.callback_pcl)

      #initial values set to center of the frame
      self.x = 320
      self.y = 240

      

  #updates self.x based on x coordinate of face
   def callback_x(self, x):
        self.x = x.data

  #updates self.y based on y coordinate of face
   def callback_y(self, y):
        self.y = y.data
        

  #publishes depth of the face in m at the x and y coordinates of the face
   def callback_pcl(self, pcl):
        
        #finds the point on the pointcloud where the face is
        point_cloud = pc2.read_points(pcl, field_names='z', skip_nans=False, uvs=[(int(round(self.x)), int(round(self.y)))])
        depth = next(point_cloud)[0]

        print(depth)

        #creates and publishes face_depth message
        depth_msg = Float64()
        depth_msg.data = depth
        self.face_depth_pub.publish(depth_msg)
      





def main():
  #inits and spins node
  rospy.init_node('face_depth_pub', anonymous=True)
  depth = Depth()
  try:
    print("spinning")
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
