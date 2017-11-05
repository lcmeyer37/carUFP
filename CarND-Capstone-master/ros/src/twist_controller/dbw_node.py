#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import collections
from twist_controller import Controller



KsOfPid = collections.namedtuple('KsOfPid', 'Kp Ki Kd')  # Data structure for holding PID gains

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        throttle_ksofpid = KsOfPid(rospy.get_param('~throttle_Kp', 0.0),
                               rospy.get_param('~throttle_Ki', 0.0),
                               rospy.get_param('~throttle_Kd', 0.0))
        steering_ksofpid = KsOfPid(rospy.get_param('~steering_Kp', 0.0),
                               rospy.get_param('~steering_Ki', 0.0),
                               rospy.get_param('~steering_Kd', 0.0))

        # Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Pass params to `Controller` constructor
        # self.controller = TwistController(<Arguments you wish to provide>)
        minSpeed=1.0*0.447
        self.controller = Controller(wheelBase=wheel_base, steerRatio=steer_ratio, minimumSpeed=minSpeed,
                                     maximumLateralAcceleration=max_lat_accel, maximumSteeringAngle=max_steer_angle,
                                     throttleKsOfPid=throttle_ksofpid, steeringKsOfPid=steering_ksofpid,
                                     accelerationLimit=accel_limit, deaccelerationLimit=decel_limit)

        # Subscriptions
        rospy.Subscriber('/dbw_enabled', Bool, self.dbwEnabledCallback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.currentVelocityCallback, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twistCommandCallback, queue_size=1)

        # Member vars
        self.dbwEnabled = True
        self.currentVelocity = None
        self.twistCommand = None
        self.loop()


    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
        
            #Safe-check
            if self.twistCommand is None or self.currentVelocity is None:
                continue

            #Using PID Controller to predict next values for Throttle, Brake and Steering, to be published to throttle_cmd, brake_cmd and steering_cmd as ThrottleCmd, BrakeCmd and SteeringCmd types of messages
            throttle, brake, steering = self.controller.control(self.twistCommand.twist.linear,
                self.twistCommand.twist.angular,
                self.currentVelocity.twist.linear,
                self.dbwEnabled)

            if self.dbwEnabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        #Normally a car is driven with either the gas or the brake pedal pressed. This way, the throttle command with steering will be sent OR the brake command with the steering. Never throttle and brake at the same time
        if throttle != 0:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)
            #rospy.loginfo("Throttle %f and Steering %f being published...", throttle, steer)
        else:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)
            #rospy.loginfo("Brake %f and Steering %f being published...", throttle, steer)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)


    def dbwEnabledCallback(self, msg):
        self.dbwEnabled = msg

    def currentVelocityCallback(self, msg):
        self.currentVelocity = msg

    def twistCommandCallback(self, msg):
        self.twistCommand = msg



if __name__ == '__main__':
    DBWNode()
