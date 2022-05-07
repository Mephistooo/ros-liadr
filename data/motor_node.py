#!/usr/bin/env python3
import rospy
import time
import math

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value

class Driver:
    def __init__(self):
        rospy.init_node('driver')
        motor_driver = Adafruit_MotorHAT(i2c_bus=1)
        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 0.5)
        self._wheel_base = rospy.get_param('~wheel_base', 0.2)

        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = motor_driver.getMotor(1)
        self._right_motor = motor_driver.getMotor(2)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity twist message
        rospy.Subscriber(
            'cmd_vel', Twist, self._velocity_received_callback)

    def _velocity_received_callback(self, message):
        """Handle new velocity command message."""
	
        self._last_received = rospy.get_time()

        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (100 * left_speed/self._max_speed)
        self._right_speed_percent = (100 * right_speed/self._max_speed)

    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
           
            if delay < self._timeout:
                self.move(1, self._left_speed_percent)
                self.move(2, self._right_speed_percent)
            else:
                self.all_stop()
                self.all_stop()
                
            rate.sleep()
   
            
    def move(self, motor_id, speed_percent):
        # speed = _clip(abs(speed_percent), 0, 100)
        motor = self._left_motor if motor_id == 1 else self._right_motor
        # Positive speeds move wheels forward, negative speeds
        # move wheels backward
        prece = int(speed_percent / 255 * 100)
        speed = int(min(max(abs(speed_percent * 255), 0), 255))
        print(prece, speed)
        motor.setSpeed(180 + abs(speed))
        if speed > 0:
            motor.run(Adafruit_MotorHAT.FORWARD)
        else:
            motor.run(Adafruit_MotorHAT.BACKWARD)
            
    
    def all_stop(self):
        self._left_motor.setSpeed(0)
        self._right_motor.setSpeed(0)
        self._left_motor.run(Adafruit_MotorHAT.RELEASE)
        self._right_motor.run(Adafruit_MotorHAT.RELEASE)




def main():
    try : 
        driver = Driver()
        # Run driver. This will block
        driver.run()
    except KeyboardInterrupt:     
        all_stop()




if __name__ == '__main__':
    main()


# # sets motor speed between [-1.0, 1.0]
# def set_speed(motor_ID, value):
#     max_pwm = 255
#     speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

#     if motor_ID == 1:
#         motor = motor_left
#     elif motor_ID == 2:
#         motor = motor_right
#     else:
#         rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d',
#                        motor_ID, value, motor_ID)
#         return

#     motor.setSpeed(speed)

#     if value > 0:
#         motor.run(Adafruit_MotorHAT.FORWARD)
#     else:
#         motor.run(Adafruit_MotorHAT.BACKWARD)

# # stops all motors


def all_stop():
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)

    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)
