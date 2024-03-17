# Nemo_AUV Package for Manual and Autonomous Control of Autonomous Underwater Vehicle (AUV)

## Project Brief

In 10 weeks, I updated a low-cost underwater vehicle, ported an existing feature-based visual SLAM package into ROS2 Iron, tuned it for use in an underwater environment, and developed a ROS2 package that interfaces with ArduSub to autonomously navigate waypoints underwater. You can read more about this project here: https://ishani-narwankar.github.io/projects/01-auv

ROS2 package for a ROV to autonomously navigate waypoints and avoid obstacles underwater.

## Package Dependencies:
### ORB SLAM 2
This repo makes use of the ORB_SLAM2 package which can be found here: https://github.com/raulmur/ORB_SLAM2.

Note that this package was made for ROS1. To use it with ROS2, you need the package below as well.

### ros2 ORB SLAM 2
The ros2_ORB_SLAM2 package can be found here: https://github.com/alsora/ros2-ORB_SLAM2/tree/master

Note that this package only works with ROS2 distros up to ROS2 Foxy. In order to use this with ROS2 Iron, I needed to rewrite some parts of this package.

## Mapping with SLAM
I am using Simultaneous Localization and Mapping (SLAM) to create a map of the underwater environment. 

In order to accomplish this, I have ported a ros1 SLAM package (ORB_SLAM2) to work with ros2 iron (which I have running on my computer). The packages necessary for this are listed in the `Package Dependencies` section.


## Launch
### Terminal 1
1. source `orbslam` workspace
2. ```ros2 run ros2_orbslam mono ~/ws/orbslam/src/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ws/orbslam/src/ros2-ORB_SLAM2/src/monocular/[camera config yaml]```
3. once map gui pops up run commands in terminal 2

### Terminal 2
*to run test orbslam on a livestream from laptop camera*

- ```ros2 run image_tools cam2image --device_id 1 --ros-args --remap /image:=/camera```

*to run test orbslam on a rosbag file*

- cd into rosbag folder
- ```ros2 bag play [rosbag file folder] --remap /video_frames:=/camera /camera_info:=/camera/camera_info```

*to run test orbslam on a livestream from realsense camera plugged into laptop*
- `ros2 run realsense2_camera realsense2_camera_node --ros-args --remap /color/image_rect_raw:=/camera`

## Notes
make sure to remap the following topics

- ```/video_frames:=/camera``` (video_frames to camera)
- ```/camera_info:=/camera/camera_info``` (camera_info to camera/camera_info)

## Debugging Notes
The following checks might be helpful for debugging if nothing shows up in the mapping gui:

- `ros2 topic info [topic name]` - to find out what type the topic is and make sure it matches with what ORB_SLAM wants for their `/camera` and `/camera/camera_info topics`

- `ros2 topic echo [topic name]` - echo the remapped topics to see if information is being published on them
