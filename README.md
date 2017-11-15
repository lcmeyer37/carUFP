# Udacity Final Project
## Programming a Real Self-Driving Car

<p align="center">
	<img width="650" height="450" src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/self-driving.png"><br>
	<img width="550" height="100" src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/logo.png">
</p>

| Team Members | Email                    | Role                                         | Slack       | github                                        |
|--------------|--------------------------|----------------------------------------------|-------------|-----------------------------------------------|
| Alan Hekle   | alanhekle@hotmail.com    | Team Leader / Corrections / PEP8 / Write-up  | @theaisback | [TheAisBack](https://github.com/TheAisBack)   |
| Samir Haddad | samir.haddad@outlook.com | TL Detector / Twist Controller               | @sam01      | [SamH1](https://github.com/SamH1)             |
| Kev Lai      | kevlai22@uw.edu          | Waypoint / Twist Controller                  | @kevlai     | [kevguy](https://github.com/kevguy)           |
| Brian McHugh | Brian.L.McHugh@gmail.com | Continous Integration / Floater              | @mchugh     | [Brian-Leary](https://github.com/Brian-Leary) |
| Lucas Meyer  | lc_meyer@hotmail.com     | TL Detector / Twist Controller/ Waypoint     | @lcmeyer    | [lcmeyer37](https://github.com/lcmeyer37)     |

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

# Final Project

Here is the architecture diagram of the self-driving car, which will help us create the software to run the actual car.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/Final-Poject-ROS.png">
</p>

# Traffic Light Detector

The Traffic Light Detector Node contains data from image_color, current_pose and base_waypoints. This node then publishes these locations to the traffic_waypoint, so that the car knows where these traffic lights are. The following image below describes these steps.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/TL-Detector.png">
</p>

### def imageColorCallback(self, msg):
This function helps find the color of the traffic light, so that the software can therefore use this information in other functions to trigger acceleration, deceleration, etc.

### def get_closest_waypoint(self, pose):
This function finds the closet waypoint for the car, this will help the vehicle to make sure its on course and find the next traffic light.

### def get_light_state(self, light):
This function helps to establish which light states which action the car shall do.

### def process_traffic_lights(self):
This function helps in establishing where the closest visible traffic light is located, if the traffic light exist.

# Waypoint Updater

The Waypoint Updater Node purpose is to update the target velocity of each waypoint which bases this information off of the traffic light and obstacle detection data. The node takes the data from base_waypoints, current_pose, obstacle_waypoints and traffic_waypoints and publishes these waypoints to the final waypoints. The following image below describes these steps.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/Waypoint-Updater.png"><br>
</p>

### def __init__(self):
Adding the subscirbers and variables that are helpful in generating waypoints for the car.

### def currentVelocityCallback(self, msg):
Calls back the current velocity of the car.

### def currentPoseCallback(self, msg):
Calls back the current position of the car.

### def baseWaypointsCallback(self, msg):
Calls back the current base way points of the car.

### def trafficStateCallback(self, msg):
Calls back the traffic light states for the car. This helps the car in knowing when to stop or go.

### def publishNextWaypoints(self):
This function is the most important code of the waypoint_updater. This code helps in publishing the waypoints for the car. The code looks at the current position, finding the closest waypoint, checking the traffic lights, the current velocity of the car, preventing the car from hitting a vehicle, limiting the car distance from other cars and imposing the velocity for the car.

# Twist Controller

With the DBW node established in the code, the twist controller is the code that needed to be edited, however, these two files are very important since they are responsible for controlling the vehicle. The dbw_node subscribes to the current_velocity, twist_cmd and the dbw_enabled. These three topics receive the vehicles linear and angular velocities, as well as, the drivers control. The node will then publish this information to the throttle_cmd, steering_cmd and the brake_cmd.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/DBW-Node.png">
</p>

### def __init__(self, *args, **kwargs):
This init function is important to point out as we are establishing the yaw_control (speed, steering, etc.), as well as, the deceleration and acceleration.

### def control(self, *args, **kwargs):
This function changes the arguments of throttle, brake and steer to your liking.

# Simulator Output

Change this once file is uploaded

[![Everything Is AWESOME](https://img.youtube.com/vi/StTqXEQ2l-Y/0.jpg)](https://www.youtube.com/watch?v=StTqXEQ2l-Y "Everything Is AWESOME")

# Real Life Output

Change this once file is uploaded

[![Everything Is AWESOME](https://img.youtube.com/vi/StTqXEQ2l-Y/0.jpg)](https://www.youtube.com/watch?v=StTqXEQ2l-Y "Everything Is AWESOME")

# How to Install Software

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir.
  [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  [Virtual Box](https://www.virtualbox.org/wiki/Downloads)
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

# Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

If the above terminal code does not work type...
```bash
sudo docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

# Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

# Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
