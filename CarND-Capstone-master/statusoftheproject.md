# Status Of The Project

This file contains considerations for the current status of the project, and present ideas and tools used for its resolution.

## About

The objective of this project is to use different concepts learned and applied during the course of the Self-Driving Car Engineer Nanodegree Program to implement a car that is able to drive itself. Traffic lights, translation and rotation variables are taken into consideration, and also the comfort of the passangers inside the car, such as maximum jerk, acceleration and velocity values were applied while implementing the car controls.

The project uses the GitHub repository provided by Udacity [here](https://github.com/udacity/CarND-Capstone), a [VM](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/7e3627d7-14f7-4a33-9dbf-75c98a6e411b/concepts/8c742938-8436-4d3d-9939-31e40284e7a6?contentVersion=1.0.0&contentLocale=en-us) provided by the instructors and a [Unity3D simulator](https://github.com/udacity/CarND-Capstone/releases) that presents the highway, traffic lights, and the car as testing environment for the code.

This project makes use of the Robot Operating System (ROS), a middleware collection of frameworks for robot software development. For this project a [system architecture](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/455f33f0-2c2d-489d-9ab2-201698fbf21a) was followed, intepreting the car controller as subsystems: Perception, Planning and Control, and each subsystem is created making use of Python scripts, ROS nodes and topics, and other provided code from the repository.

## System Architecture Implementation

The System Architecture comprises of the following subsystems:

- Perception: subsystem responsible for the detection of the traffic lights, the car should be able to visualize the traffic light signs and characterize it as a green, red or yellow light. The folder tl_detector in this repo contains the files needed for implementation of this subsystem:
    
    - tl_detector.py: this python script is basically responsible for calling the TLClassifier class, that is able to understand the traffic light signs, and contains the following ROS subscribers and publishers:
      - '/current_pose' subscription, current position of the car.
      - '/base_waypoints' subscription, waypoints of the simulator track.
      - '/vehicle/traffic_lights' subscription, location of the traffic lights in 3D map space.
      - '/image_color' subscription, contains the image captured by the car's camera for traffic light classification.
      - '/all_traffic_waypoint' publisher, contains the status of the traffic lights.
    
    - light_classification/tl_classifier.py: python script that implements the TLClassifier class, the following methods for this class are worth mentioning:
      - def setupGraphForTrafficLightLocalization(self): this function is responsible for loading a Deep Neural Network that is being used for the traffic lights classification, the SSD w/ MobileNet trained with the COCO dataset. This neural network is presented by the [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection), and contains dozens of different object classes for detection. For this project, just class '10' is of interest, the traffic light object class.
      - def locateTrafficLightsOnFrame(self, image, visual=False): responsible for using the image provided by the car's camera to locate the traffic lights on it, making use of the Neural Network described above. This function returns rectangles, that surround and gives information of the position of the traffic light detected.
      - def classifyTrafficLightState(self,image): using the rectangle provided above, the tl_detector.py script separates and resizes a new image 32x32 that contains just the traffic light sign. That image is used in this function, that basically applies three HSV masks for green, yellow and red. Then, these masks have their pixels counted, and the mask that has more pixels probably represents the color of the traffic light. For example, if the mask_red has more pixels than mask_yellow or mask_green, probably the light is red.
      
- Planning: subsystem responsible for taking into consideration the information provided by the Perception subsystem, and current car information, to decide a further course of action for the car. The folder waypoint_updater contains one python script used for that matter.
  
  - waypoint_updater.py: this python script uses the following:
    - '/current_pose' subscription.
    - '/base_waypoints' subscription.
    - '/all_traffic_waypoint' subscription.
    - '/current_velocity' subscription, contains the current velocity of the car.
    - '/final_waypoints' publisher, contains the list of waypoints that are given to the Control subsytem, that uses it to drive the car.
    - def publishNextWaypoints(self): that function is worth mentioning, it basically uses the subscriptions described above to return the list of waypoints to be published to '/final_waypoints'. This function starts by finding the closest waypoint to the vehicle, next this waypoint is considered first in the following list of next waypoints, then the traffic light location and its sign status is used to further decide if the car should be stopping, or accelerating, or maintaing velocity. Finally, these decisions are all implemented as the next waypoints list that is returned by this function.
    
- Control: subsystem responsible for using the waypoints provided by the Planning subsystem, and current car information, to publish messages that are used by the simulator, related directly to control of the car. The folder twist_controller contains python scripts related to this subsystem:

  - dbw_node.py: this python script uses several ros parameters, subscribers and publishers such as:
    - '~vehicle_mass': contains the car mass.
    - '~fuel_capacity': fuel capacity of the car.
    - '~max_lat_accel': maximum lateral acceleration of the car.
    - '~max_steer_angle': maximum steering angle of the car.
    - '~throttle_Kp': Kp value for PID controller of the throttle.
    - '~throttle_Ki': Ki value for PID controller of the throttle.
    - '~throttle_Kd': Kd value for PID controller of the throttle.
    - '/current_velocity' subscription.
    - '/dbw_enabled' subscription, contains information if the Drive-by-wire (DBW) system is enabled.
    - '/twist_cmd' subscription, published by the waypoint_follower/pure_pursuit.cpp provided script.
    - '/vehicle/steering_cmd' publisher, contains a SteeringCmd type ROS message, that is used by the simulator for steering of the car.
    - '/vehicle/throttle_cmd' publisher, contains a ThrottleCmd type ROS message, that is used by the simulator for throttling of the car.
    - '/vehicle/brake_cmd' publisher, contains a BrakeCmd type ROS message, that is used by the simulator for braking of the car.
    - def publish(self, throttle, brake, steer): worth mentioning, this function uses the throttle, brake and steering variables provided by the Controller class from twist_controller.py to preprocess and decide which control variables are of interest and then published. For example, normally a driver wouldn't use the brake and throttle pedals at the same time. So, in this function, it first sees if the throttle provided by the controller in non-zero, if so, just throttle and steering are published. Otherwise, just the brake command will be published along with the steering.
  
  - twist_controller.py: this python script uses the PID controller provided by Udacity, the Proportional-Integral-Derivative controller is a control loop feedback mechanism used in industrial control systems that require continuously modulated control. The PID controller calculates an error value as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral and derivative terms. This controller is one of the topics explained on this course on Term 2: Sensor Fusion, Localization and Control.
    - def control(self, twistCommandLinear, twistCommandAngular, currentVelocityLinear, dbwEnabled): this function is worth mentioning, it calculates the errors described above and uses the PID class provided to calculate the next value for throttle and brake. The steering value uses the YawController class to get the steering angle needed for driving.
    
    
## Considerations

Considering that the car currently is able to drive itself along the simulator, with an average speed of approximately 10 miles per hour, and that the car is able to follow the path of the highway and detect traffic lights, deciding if it should stop or go along the course, the group has decided that the current state of the project is acceptable as resolution and for presentation.

November 18th, 2017. Roadhammer group.
