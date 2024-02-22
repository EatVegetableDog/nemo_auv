# Nemo_AUV (In-Progress)

### Current Step in Project: 
Creating underwater map using ORB_SLAM2 ported into ros2 iron (can read more about how I did this in the `Mapping` section below)

## Project Brief

This was proposed and developed as an independent 2024 winter quarter research project as a part of Northwestern's MS in Robotics program.

ROS2 package for a ROV to autonomously navigate waypoints and avoid obstacles underwater.

![btest](https://github.com/ishani-narwankar/nemo_auv/assets/42013894/ae804de7-0491-43ce-803a-b019aff8de04)

## Goals

- Improve mechanical and electrical design of low-cost AUV
- Autonomous depth control
- Autonomous waypoint navigation

    - Create map of underwater environment
    - Add digital waypoints to map
    - Navigate to digital waypoints

- Obstacle avoidance

## Mapping with SLAM
I am using Simulated Learning and Mapping (SLAM) to create a map of the underwater environment. 

In order to accomplish this, I have ported a ros1 SLAM package (ORB_SLAM2) to work with ros2 iron (which I have running on my computer). 

You can find the info on how I set that up here: https://github.com/ishani-narwankar/orbslam_iron_nemo



<!-- ## Project Goals
- Fallback goals:
- Core goals:
- Stre -->

<!-- ## Techniques/External Packages Utilized for Achieving Project Goals
- Simulated Learning and Mapping (SLAM) -->
