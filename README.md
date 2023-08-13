# Robotic Eye
Design and Evaluation of a Bioinspired Tendon-Driven 3D-Printed Robotic Eye with Active Vision Capabilities

Download the XL_320 Toolbox [here](https://github.com/hamidosooli/XL_320Toolbox).

## Robotic Eye ROS Package

1. Overview

The robotic_eye_ros_pkg directory contains the ROS package for using the Eye Robot. With this package, the robot is able to track any face within the camera's field of vision and center that face on the frame. The robot is also able to estimate the depth of the face using stereo vision.



&nbsp;




2. Launch Files

There are 2 ROS launch files locates within the launch folder of the repository

depth.launch - This launch file runs the necessary nodes to get an estimation of the depth (in metres) of any face in the frame. When launched, a frame will pop up where all the faces in the image will be labeled with a green box and the estimated depth of that face in metres. The stereo vision parameters can be edited within this launch file.

face_tracking.launch - This launch file runs the necessary nodes to track any face within the frame. When launched, the robot will start moving the eyes to center the first face it detects within the frame, again labeling it with a green box


&nbsp;


3. Additional Features

The calibration data folder contains the necessary calibration data required for stereo rectification. The weights folder contains the weights for the yolov5 face detection model used to detect faces. 


&nbsp;

4. Nodes

left_display - publishes the image feed and calibration data for the left camera

right_display - publishes the image feed and calibration data for the right camera

face_detector - subscribes to left image feed, calibration data, and the face depth and detects faces labeling them with a green box and the depth estimation 

depth_node - subcribes to the pointcloud publication from the ros stereo_image_proc node to publish the face depth in metres

eye_mover - moves the robot based on the x and y control effort published by the ros pid package


&nbsp;

5. Additional scripts

center.py - a python executable file that sets all the servo positions to 512 to center the eyes

robot.py - contains a robot class to organize all the hardware writes

## Citation

If you found robotic eye to be useful, we would really appreciate if you could cite our work:

- [1] Hamid Osooli, Mohsen Irani Rahaghi, S. Reza Ahmadzadeh, "Design and Evaluation of a Bioinspired Tendon-Driven 3D-Printed Robotic Eye with Active Vision Capabilities," In Proc.  20th International Conference on Ubiquitous Robots (UR 2023), Honolulu, Hawaii, pp. xxx-xxx, Jun. 25-28, 2023

```bibtex
@inproceedings{osooli2023design,
  title={Design and Evaluation of a Bioinspired Tendon-Driven 3D-Printed Robotic Eye with Active Vision Capabilities},
  author={Osooli, Hamid and Rahaghi, Mohsen Irani and Ahmadzadeh, S Reza},
  booktitle={2023 20th International Conference on Ubiquitous Robots (UR)},
  pages={747--752},
  year={2023},
  organization={IEEE}
}

```

- [2] Hamid Osooli, Amirhossein Nikoofard, and Zahra Shirmohammadi. "Game Theory for Eye Robot Movement: Approach and Hardware Implementation." In 2019 27th Iranian Conference on Electrical Engineering (ICEE 2019), Yazd, Iran, pp. 1081-1085. IEEE, 2019, DOI: 10.1109/IranianCEE.2019.8786637

```bibtex
@inproceedings{osooli2019game,
  title={Game Theory for Eye Robot Movement: Approach and Hardware Implementation},
  author={Osooli, Hamid and Nikoofard, Amirhossein and Shirmohammadi, Zahra},
  booktitle={2019 27th Iranian Conference on Electrical Engineering (ICEE)},
  pages={1081--1085},
  year={2019},
  organization={IEEE},
  doi={10.1109/IranianCEE.2019.8786637}
}

```


