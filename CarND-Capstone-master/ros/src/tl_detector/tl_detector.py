#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, TrafficLightTwo, Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import time
import numpy as np

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.sim_testing = bool(rospy.get_param("~sim_testing", True))
        threshold = rospy.get_param('~threshold', 0.3)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCallback)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.baseWaypointsCallback)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and helps you acquire an accurate ground truth data source for the traffic light classifier by sending the current color state of all traffic lights in the simulator. When testing on the vehicle, the color state will not be available. You'll need to rely on the position of the light and the camera image to predict it.

        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.trafficLightsCallback)
        sub6 = rospy.Subscriber('/image_color', Image, self.imageColorCallback, queue_size=1, buff_size=2*52428800) # extended

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        #Not being used. /all_traffic_waypoint publishes status for all traffic lights present.
        #self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.upcoming_traffic_light_pub = rospy.Publisher('/all_traffic_waypoint', TrafficLightTwo, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(threshold)
        self.listener = tf.TransformListener()

        #Creating a null image, to first initiate the TL localization algorithm, testing for a no-detecion result.
        img_full_np = np.asarray(np.zeros((800,600,3)), dtype="uint8")
        self.light_classifier.locateTrafficLightsOnFrame(img_full_np)


        self.state = None
        self.last_state = None
        self.last_wp = -1
        self.state_count = 0
        print("TLDetector instance initialization complete...")
        print("[Please, start Carla Unity Simulator now if desired]")

        rospy.spin()

    def currentPoseCallback(self, msg):
        self.pose = msg

    def baseWaypointsCallback(self, waypoints):
        self.waypoints = waypoints.waypoints

    def trafficLightsCallback(self, msg):
        self.lights = msg.lights

    def imageColorCallback(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = state
            self.last_wp = light_wp
            tl_status_msg = TrafficLightTwo()
            tl_status_msg.header.frame_id = '/world'
            tl_status_msg.header.stamp = rospy.Time.now()
            tl_status_msg.waypoint = light_wp
            tl_status_msg.state = state
            self.upcoming_traffic_light_pub.publish(tl_status_msg)
            #rospy.loginfo("Traffic Light waypoint is " + light_wp + " and has state " + self.state)
        else:
            # we keep publishing the last state and wp until it gets confirmed.
            tl_status_msg = TrafficLightTwo()
            tl_status_msg.header.frame_id = '/world'
            tl_status_msg.header.stamp = rospy.Time.now()
            tl_status_msg.waypoint = self.last_wp
            if self.last_state is not None:
                tl_status_msg.state = self.last_state
            else:
                tl_status_msg.state = TrafficLight.RED
            #rospy.loginfo("Traffic Light waypoint is " + self.last_wp + " and has state " + self.state)
            self.upcoming_traffic_light_pub.publish(tl_status_msg)
        self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints is None:
            return

        minimumDistance = 10000
        minimumDistanceIndex = None

        positionX = pose.position.x
        positionY = pose.position.y

        #Minimum distance calculation between points loop.
        for i, waypoint in enumerate(self.waypoints):
            waypointPositionX = waypoint.pose.pose.position.x
            waypointPositionY = waypoint.pose.pose.position.y
            distanceCalculated = math.sqrt((positionX - waypointPositionX)**2 + (positionY - waypointPositionY)**2)
            if (distanceCalculated < minimumDistance):
                minimumDistanceIndex = i
                minimumDistance = distanceCalculated

        return minimumDistanceIndex


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #Return traffic light state as red if TLClassifier object is not ready yet.
        if self.light_classifier is None:
            return TrafficLight.RED

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification

        #1 - Localization of Traffic Lights
        #Get image provided
        processed_img = cv_image.copy()
        processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2RGB)
        light_state = TrafficLight.UNKNOWN
        currentTrafficLightState = None

        #Finds the light being processed at the moment, by calculating its distance to the other traffic lights provided
        for tl in self.lights:
            dist = math.sqrt((tl.pose.pose.position.x - light.position.x)**2 + (tl.pose.pose.position.y - light.position.y)**2)
            if (dist < 50):
                currentTrafficLightState = tl.state #once traffic light is found on the list, pass its state to currentTrafficLightState variable
                break

        #Finds localization of the traffic light on the image inputted.
        img_full_np = np.asarray(processed_img, dtype="uint8")
        trafficLightBox = self.light_classifier.locateTrafficLightsOnFrame(img_full_np)


        #2 - Use localization of traffic light for classification of its light current state

        unknown = False
        #If the box is [0,0,0,0], meaning no detection was made
        if np.array_equal(trafficLightBox, np.zeros(4)):
            unknown = True

        #If the box is holding the localization of a traffic light
        else:
            #resize image of the traffic box to 32x32 pixels, for counting of Red, Yellow and Green pixels for classification
            img_np = cv2.resize(processed_img[trafficLightBox[0]:trafficLightBox[2], trafficLightBox[1]:trafficLightBox[3]], (32, 32))
            self.light_classifier.classifyTrafficLightState(img_np)
            light_state = self.light_classifier.signal_status

        return light_state




    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        closestTrafficLightStopWaypointIndex = None
        distanceToTrafficLight = 10000   #initialize to high value

        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        #Find the closest visible traffic light (if one exists)
        for stopLinePosition in stop_line_positions:

            lightStopPose = Pose()
            lightStopPose.position.x = stopLinePosition[0]
            lightStopPose.position.y = stopLinePosition[1]
            trafficLightClosest = self.get_closest_waypoint(lightStopPose) #Found closest TL

            if trafficLightClosest >= car_position :    #Found a waypoint right in front of the car

                if closestTrafficLightStopWaypointIndex is None:
                    closestTrafficLightStopWaypointIndex = trafficLightClosest
                    light = lightStopPose

                #Check light waypoint, if its closer to the car to process
                elif trafficLightClosest < closestTrafficLightStopWaypointIndex:
                    closestTrafficLightStopWaypointIndex = trafficLightClosest
                    light = lightStopPose

        if ((car_position is not None) and (closestTrafficLightStopWaypointIndex is not None)):
            distanceToTrafficLight = abs(car_position - closestTrafficLightStopWaypointIndex)
            #rospy.loginfo("Nearest traffic light position is %s", closestTrafficLightStopWaypointIndex)

        #If traffic light is considered close enough. Start its classification.
        if light and distanceToTrafficLight < 100:
            state = self.get_light_state(light)
            return closestTrafficLightStopWaypointIndex, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
