#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightTwo
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math, sys
from itertools import islice, cycle

ONE_MPH = 0.44704
numberOfWaypointsToPublish = 250 # Number of waypoints to be published
maximumSpeed = 10*ONE_MPH #maximum speed of the car, in meters per second (mph * 0.44704 = speed in m/s)
numberWaypointsDeaccelerationTF = 75 #Number of waypoints to slowdown, getting to a traffic light

#Waypoint Updater Node Overview
#https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/e1f2a5cf-c697-4880-afb2-b88f3f83d07b

class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.currentPoseCallback, queue_size=1, buff_size=512*1024)
        rospy.Subscriber('/base_waypoints', Lane, self.baseWaypointsCallback)
        rospy.Subscriber('/all_traffic_waypoint',TrafficLightTwo,self.trafficStateCallback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.currentVelocityCallback, queue_size=1)

        self.simulationTesting = bool(rospy.get_param("~sim_testing", True))

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints = None #waypoints on the road (waypoints)
        self.currentPose = None #holds the current pose information (current_pose)
        self.currentVelocity = None #holds the current vel. information (current_velocity)
        self.redLightWaypoint = -1
        self.closestWaypointIndex = None #holds the index for the closest waypoint from the vehicle
        self.nextTrafficLightState = None #holds the state of the next traffic light
        self.nextTrafficLightWaypointIndex = None #holds the waypoint index where the next traffic light is
        rospy.spin()

    'currentVelocityCallback, callback function for current_velocity subscriber'
    def currentVelocityCallback(self, msg):
        self.currentVelocity = msg

    'currentPoseCallback, callback function for current_pose subscriber'
    def currentPoseCallback(self, msg):
        self.currentPose = msg.pose
        self.publishNextWaypoints()

    'baseWaypointsCallback, callback function for base_waypoints subscriber'
    def baseWaypointsCallback(self, msg):
        rospy.loginfo('WPUpdater: Got initial waypoints')
        if self.waypoints is None:
            self.waypoints = msg.waypoints
            self.publishNextWaypoints()

    'trafficStateCallback, callback function for all_traffic_waypoint subscriber'
    def trafficStateCallback(self, msg):
        if ((self.nextTrafficLightWaypointIndex is None) and (msg.waypoint == -1)):
            return
		
        self.nextTrafficLightWaypointIndex = msg.waypoint
        self.nextTrafficLightState = msg.state
		
        if ((msg.state == TrafficLight.RED) or (msg.state == TrafficLight.YELLOW)):
            self.redLightWaypoint = msg.waypoint
        else:
            self.redLightWaypoint = -1
	
        self.publishNextWaypoints()

    'publishNextWaypoints, function that uses the messages provided by the subscribed topics base_waypoints, current_pose and etc. To generate a list of waypoints and publish those to the final_waypoints topic (this is a Lane message)'
    def publishNextWaypoints(self):

        ###CHECK IF THERE IS SOME INCONSISTENCY WITH SOME VARIABLE BEFORE PROCEEDING
        if self.waypoints is None or self.currentVelocity is None or self.currentPose is None or self.nextTrafficLightWaypointIndex is None or self.redLightWaypoint is None:
            return


        ###CALCULATING CLOSEST WAYPOINT TO THE VEHICLE
        velocityX = self.currentVelocity.twist.linear.x
        carPositionX = self.currentPose.position.x
        carPositionY = self.currentPose.position.y

        minimumDistance = 99999
        minimumDistanceIndex = None

        startIndexWaypoints = 0
        endIndexWaypoints = len(self.waypoints)

        #check if it is running in simulation mode
        if self.simulationTesting and self.closestWaypointIndex is not None:
            startIndexWaypoints = self.closestWaypointIndex - 30
            endIndexWaypoints = min(endIndexWaypoints,self.closestWaypointIndex + 30)

        #simple for loop for calculation of distance between current car position and waypoints provided, to find closest waypoint to the vehicle
        for i in range(startIndexWaypoints, endIndexWaypoints):
            waypoint = self.waypoints[i]
            waypointsPosX = waypoint.pose.pose.position.x
            waypointsPosY = waypoint.pose.pose.position.y
            distanceCalculated = math.sqrt((carPositionX - waypointsPosX)**2 + (carPositionY - waypointsPosY)**2)

            if distanceCalculated < minimumDistance:
                minimumDistance = distanceCalculated
                minimumDistanceIndex = i

        closestWaypoint = self.waypoints[minimumDistanceIndex]
        closestWaypointPosition = closestWaypoint.pose.pose.position
        self.closestWaypointIndex = minimumDistanceIndex
        rospy.loginfo("Running... [Closest Waypoint calculated: index %d, positionX %f, positionY %f]", minimumDistanceIndex, closestWaypointPosition.x, closestWaypointPosition.y)

        #GENERATE WAYPOINTS AND PUBLISH THEM BASED ON TRAFFIC LIGHT CLASSIFICATIONS
        #https://docs.python.org/2/library/itertools.html#itertools.islice

        #create list for next waypoints for the car to follow, it starts with the waypoint that's closest to the vehicle, plus the number of waypoints desired to be published (ex: 200)
        nextWaypoints = list(islice(cycle(self.waypoints), minimumDistanceIndex, minimumDistanceIndex + numberOfWaypointsToPublish - 1))

        currentVelocity = closestWaypoint.twist.twist.linear.x
        numberWaypointsToTF = self.nextTrafficLightWaypointIndex - minimumDistanceIndex

        isTrafficLightNear = numberWaypointsToTF < numberWaypointsDeaccelerationTF
        isRedTrafficLightNear = (self.redLightWaypoint != -1 and isTrafficLightNear)

        #check if red traffic light is too close to the vehicle, if it is unconsider publishing further waypoints at the moment
        if isRedTrafficLightNear and self.redLightWaypoint <= minimumDistanceIndex:
            return

        #otherwise, iterate through the next waypoints list
        for i in range(len(nextWaypoints) - 1):

            securityMargin = 15
            waypointToGo = self.nextTrafficLightWaypointIndex - minimumDistanceIndex - i - securityMargin

            #if red traffic light is not near the car
            if not isRedTrafficLightNear:

                #if current velocity of the car is considerable (larger than 3mph) and the waypointToGo calculated is in the close range of the car
                if velocityX > 3*ONE_MPH and waypointToGo < 30 and waypointToGo > 0:
                    #change car speed to 5mph
                    nextWaypoints[i].twist.twist.linear.x = 5*ONE_MPH
                #else, if the waypointToGo is not in a close range, accelerate the car, saying that this waypoint can run in maximum velocity
                else:
                    nextWaypoints[i].twist.twist.linear.x = maximumSpeed

            #else, if red traffic light is near the car
            else:

                #if the range of waypointToGo calculated is really close to the car, < 4 index, stop the car
                if waypointToGo < 4:
                    newVelocity = 0.0
                #if waypoint is still close, change car speed to 5mph
                elif waypointToGo < 30:
                    newVelocity = 5*ONE_MPH
                #if the range is larger than 30, change velocity in a constant pattern, linear calculation below for rate of change of speed.
                else:
                    newVelocity = maximumSpeed - (numberWaypointsDeaccelerationTF - waypointToGo)*(maximumSpeed/numberWaypointsDeaccelerationTF)
                if newVelocity < 0.1:
                    newVelocity = 0

                #impose the velocity calculated for the waypoint iterated considering the scenarios above
                nextWaypoints[i].twist.twist.linear.x = newVelocity

        #PUBLISH WAYPOINTS (this is a styx_msgs/Lane type of message)
        lane = Lane()
        lane.waypoints = nextWaypoints
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
