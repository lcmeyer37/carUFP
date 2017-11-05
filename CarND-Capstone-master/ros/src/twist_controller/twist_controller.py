from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time
import rospy


ONE_MPH = 0.44704 #1 mph = 0.44704 m/s
MAX_SPEED = 40.0 #in mph


class Controller(object):
    def __init__(self, *args, **kwargs):

        self.throttle_pid = PID(kwargs['throttleKsOfPid'])
        self.yaw_control = YawController(kwargs['wheelBase'], kwargs['steerRatio'],
                                         kwargs['minimumSpeed'], kwargs['maximumLateralAcceleration'],
                                         kwargs['maximumSteeringAngle'],
                                         kwargs['steeringKsOfPid']
                                         )
        self.lastTime = None
        self.accelerationLimit = kwargs['accelerationLimit']
        self.deaccelerationLimit = kwargs['deaccelerationLimit']
        self.filter = LowPassFilter(0.2,0.1)


    def control(self, twistCommandLinear, twistCommandAngular, currentVelocityLinear, dbwEnabled):
        if self.lastTime is None or not dbwEnabled:
            self.lastTime = rospy.get_time()
            return 0.0, 0.0, 0.0

        deltaTime = rospy.get_time() - self.lastTime

        errorTwistLinear = min(twistCommandLinear.x, MAX_SPEED*ONE_MPH) - currentVelocityLinear.x
        errorTwistLinear = max(self.deaccelerationLimit*deltaTime, min(self.accelerationLimit*deltaTime, errorTwistLinear))
        throttle = self.throttle_pid.step(errorTwistLinear, deltaTime)
        throttle = max(0.0, min(1.0, throttle))
        if errorTwistLinear < 0:
            brake = -15.0*errorTwistLinear
            brake = max(brake, 1.0)
            throttle = 0.0
        else:
            brake = 0.0

        if abs(twistCommandLinear.x) < 0.1:
            brake = 12.0

        steer = self.yaw_control.get_steering(twistCommandLinear.x, twistCommandAngular.z, currentVelocityLinear.x)
        steer = self.filter.filt(steer)
        self.lastTime = rospy.get_time()
        return throttle, brake, steer
