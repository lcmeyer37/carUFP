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

## About

The objective of this project is to use different concepts learned and applied during the course of the Self-Driving Car Engineer Nanodegree Program to implement a car that is able to drive autonomously. Traffic lights, translation and rotation variables are taken into consideration, and also the comfort of the passengers inside the car, such as maximum jerk, acceleration and velocity values were applied while implementing the car controls.

The project uses the GitHub repository provided by Udacity [here](https://github.com/udacity/CarND-Capstone), a [VM](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/7e3627d7-14f7-4a33-9dbf-75c98a6e411b/concepts/8c742938-8436-4d3d-9939-31e40284e7a6?contentVersion=1.0.0&contentLocale=en-us) provided by the instructors and a [Unity3D simulator](https://github.com/udacity/CarND-Capstone/releases) that presents the highway, traffic lights, and the car as testing environment for the code.

This project makes use of the Robot Operating System (ROS), a middleware collection of frameworks for robot software development. For this project a [system architecture](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/455f33f0-2c2d-489d-9ab2-201698fbf21a) was followed, interpreting the car controller as subsystems: Perception, Planning and Control, and each subsystem is created making use of Python scripts, ROS nodes and topics, and other provided code from the repository.

Here is the architecture diagram of the self-driving car, which will help us create the software to run the actual car.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/Final-Poject-ROS.png">
</p>

# System Architecture Implementation

The System Architecture comprises of the following subsystems:

## Perception:

Subsystem responsible for the detection of the traffic lights, the car should be able to visualize the traffic light signs and characterize it as a green, red or yellow light. The folder tl_detector in this repo contains the files needed for implementation of this subsystem:

### tl_detector.py:

This python script is basically responsible for calling the TLClassifier class, that is able to understand the traffic light signs, and contains the following ROS subscribers and publishers:

#### '/current_pose':

Subscription, current position of the car.

#### '/base_waypoints':

Subscription, waypoints of the simulator track.

#### '/vehicle/traffic_lights':

Subscription, location of the traffic lights in 3D map space.

#### '/image_color':

Subscription, contains the image captured by the car's camera for traffic light classification.

#### '/all_traffic_waypoint':

Publisher, contains the status of the traffic lights.

### light_classification/tl_classifier.py:

Python script that implements the TLClassifier class, the following methods for this class are worth mentioning:

#### def setupGraphForTrafficLightLocalization(self):

This function is responsible for loading a Deep Neural Network that is being used for the traffic lights classification, the SSD w/ MobileNet trained with the COCO dataset. This neural network is presented by the [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection), and contains dozens of different object classes for detection. For this project, just class '10' is of interest, the traffic light object class.

#### def locateTrafficLightsOnFrame(self, image, visual=False):

Responsible for using the image provided by the car's camera to locate the traffic lights on it, making use of the Neural Network described above. This function returns rectangles, which surround and gives information of the position of the traffic light detected.

#### def classifyTrafficLightState(self,image):

Using the rectangle provided above, the tl_detector.py script separates and resizes a new image 32x32 that contains just the traffic light sign. That image is used in this function, which basically applies three HSV masks for green, yellow and red. Then, these masks have their pixels counted, and the mask that has more pixels probably represents the color of the traffic light. For example, if the mask_red has more pixels than mask_yellow or mask_green, probably the light is red.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/TL-Detector.png">
</p>

## Planning:

The subsystem is responsible for taking into consideration the information provided by the Perception subsystem, and current car information, to decide a further course of action for the car. The folder waypoint_updater contains one python script used for that matter.

### waypoint_updater.py:

This python script uses the following:

#### '/current_pose':

Subscription.

#### '/base_waypoints':

Subscription.

#### '/all_traffic_waypoint':

Subscription.

#### '/current_velocity':

Subscription, contains the current velocity of the car.

#### '/final_waypoints':

Publisher, contains the list of waypoints that are given to the Control subsystem, that uses it to drive the car.

#### def publishNextWaypoints(self):

This function is worth mentioning, it basically uses the subscriptions described above to return the list of waypoints to be published to '/final_waypoints'. This function starts by finding the closest waypoint to the vehicle, next this waypoint is considered first in the following list of next waypoints, then the traffic light location and its sign status is used to further decide if the car should be stopping, or accelerating, or maintaining velocity. Finally, these decisions are all implemented as the next waypoints list that is returned by this function.

<p align="center">
	<img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/Waypoint-Updater.png"><br>
</p>

## Control

Subsystem responsible for using the waypoints provided by the Planning subsystem, and current car information, to publish messages that are used by the simulator, related directly to control of the car. The folder twist_controller contains python scripts related to this subsystem:

### dbw_node.py:

This python script uses several ros parameters, subscribers and publishers such as:

#### '~vehicle_mass':

Contains the car mass.

#### '~fuel_capacity':

Fuel capacity of the car.

#### '~max_lat_accel':

Maximum lateral acceleration of the car.

#### '~max_steer_angle':

Maximum steering angle of the car.

#### '~throttle_Kp':

Kp value for PID controller of the throttle.

#### '~throttle_Ki':

Ki value for PID controller of the throttle.

#### '~throttle_Kd':

Kd value for PID controller of the throttle.

#### '/current_velocity':

Subscription.

#### '/dbw_enabled':

Subscription, contains information if the Drive-by-wire (DBW) system is enabled.

#### '/twist_cmd':

Subscription, published by the waypoint_follower/pure_pursuit.cpp provided script.

#### '/vehicle/steering_cmd':

Publisher, contains a SteeringCmd type ROS message, that is used by the simulator for steering of the car.

#### '/vehicle/throttle_cmd':

Publisher, contains a ThrottleCmd type ROS message, that is used by the simulator for throttling of the car.

#### '/vehicle/brake_cmd':

Publisher, contains a BrakeCmd type ROS message, that is used by the simulator for braking of the car.

#### def publish(self, throttle, brake, steer):

Worth mentioning, this function uses the throttle, brake and steering variables provided by the Controller class from twist_controller.py to preprocess and decide which control variables are of interest and then published. For example, normally a driver wouldn't use the brake and throttle pedals at the same time. So, in this function, it first sees if the throttle provided by the controller in non-zero, if so, just throttle and steering are published. Otherwise, just the brake command will be published along with the steering.

### twist_controller.py:

This python script uses the PID controller provided by Udacity, the Proportional-Integral-Derivative controller is a control loop feedback mechanism used in industrial control systems that require continuously modulated control. The PID controller calculates an error value as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral and derivative terms. This controller is one of the topics explained on this course on Term 2: Sensor Fusion, Localization and Control.

### def control(self, twistCommandLinear, twistCommandAngular, currentVelocityLinear, dbwEnabled):

This function is worth mentioning because it calculates the errors described above and uses the PID class provided to calculate the next value for throttle and brake. The steering value uses the YawController class to get the steering angle needed for driving.

<p align="center">
  <img src="https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/DBW-Node.png">
</p>

## Simulator Output

Here is our output from the simulator.

[![Self-Driving Car](https://github.com/TheAisBack/CarND-Capstone/blob/master/imgs/Self_Driving_Car.png)](https://www.youtube.com/watch?v=jIykmJD2Dl8 "Udacity Self-Driving Car")

## Final Considerations

Considering that the car currently is able to drive itself along the simulator, with an average speed of approximately 10 miles per hour, and that the car is able to follow the path of the highway and detect traffic lights, deciding if it should stop or go along the course, the group has decided that the current state of the project is acceptable as resolution and for presentation.

## How to Install Software

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

## Docker Installation
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

## Usage

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

## Real world testing
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
