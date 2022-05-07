#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from board import SCL, SDA
import busio
import time
import atexit
from adafruit_pca9685 import PCA9685
import pprint

def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:
    def __init__(self, forward_pin, backward_pin, pwm_pin):
        self.forward_pwm = forward_pin
        self.backward_pwm = backward_pin
        self.pwm_pin = pwm_pin


class Driver:
    MAX_DUTY = 65535

    def __init__(self):
        rospy.init_node('driver')
        self.i2c_bus = busio.I2C(SCL, SDA)
        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(self.i2c_bus)
        # Set the PWM frequency to 60hz.
        self.pca.frequency = 60
        atexit.register(self.clean_up_gipo)
        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 0.5)
        self._wheel_base = rospy.get_param('~wheel_base', 0.2)

        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = Motor(10, 9, 8)
        self._right_motor = Motor(12, 11, 13)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity twist message
        rospy.Subscriber(
            'cmd_vel', Twist, self._velocity_received_callback)

    def _velocity_received_callback(self, message):
        """Handle new velocity command message."""
        #print(type(message))
        self._last_received = rospy.get_time()
        #pprint(message)
        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular * self._wheel_base / 2
        right_speed = linear + angular * self._wheel_base / 2

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
            try:
                # If we haven't received new commands for a while, we
                # may have lost contact with the commander-- stop
                # moving
                delay = rospy.get_time() - self._last_received
                if delay < self._timeout:
                    self.move(1, self._left_speed_percent)
                    self.move(2, self._right_speed_percent)
                else:
                    self.move(1, 0)
                    self.move(2, 0)

                rate.sleep()
            except rospy.ROSInterruptException:
                self.clean_up_gipo()
                exit()

    def move(self, motor_id, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)
        motor = self._left_motor if motor_id == 1 else self._right_motor
        duty_cycle = int(self.MAX_DUTY / 100 * abs(speed))
        if duty_cycle != 0:
            print("Setting duty cycle to {} to motor {}".format(duty_cycle, motor_id))
        # motor.setSpeed(speed_percent / 100 * 255)
        if speed_percent > 0:
            self.pca.channels[motor.forward_pwm].duty_cycle = 65535
            self.pca.channels[motor.backward_pwm].duty_cycle = 0
            self.pca.channels[motor.pwm_pin].duty_cycle = duty_cycle
        else:
            self.pca.channels[motor.backward_pwm].duty_cycle = 65535
            self.pca.channels[motor.forward_pwm].duty_cycle = 0
            self.pca.channels[motor.pwm_pin].duty_cycle = duty_cycle

    def clean_up_gipo(self):
        for pin_1, pin_2 in zip(self._right_motor.__dict__.values(), self._left_motor.__dict__.values()):
            self.pca.channels[pin_1].duty_cycle = 0
            self.pca.channels[pin_2].duty_cycle = 0
        self.pca.deinit()
        self.i2c_bus.deinit()


def main():

    driver = Driver()
    driver.run()


if __name__ == '__main__':
    main()
