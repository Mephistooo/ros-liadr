#!/usr/bin/env python3
import rospy
import time
import math

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 255
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)


# directional commands (degree, speed)
def on_cmd_dir(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID,  1.0) 
	elif msg.data.lower() == "right":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID, -1.0) 
	elif msg.data.lower() == "forward":
		set_speed(motor_left_ID,   -1.0)
		set_speed(motor_right_ID,  1.0)
	elif msg.data.lower() == "backward":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID, -1.0)  
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)



def vel_callback(msg):
    all_stop()
    linear = msg.linear.x
    angle = msg.angular.z
    WHEEL_DIST = 16

    #rotation 
    if linear == 0 :
        spd_right = angle * WHEEL_DIST / 2.0
        spd_left = -spd_right
    # forward or backward
    elif angle == 0 :
         spd_left = spd_right = linear
    else :
        spd_left = linear - angle * WHEEL_DIST / 2.0
        spd_right = linear + angle * WHEEL_DIST / 2.0
    
    # speed_wish_right = (angle * WHEEL_DIST) / 2 + linear
    # speed_wish_left = linear * 2 - speed_wish_right
    speed_wish_left = spd_left
    speed_wish_right = spd_right
    print("{} right, {} left".format(spd_right, spd_left))
    # max_pwm = 255
    # speed = int(min(max(abs(spd_right * max_pwm), 0), max_pwm))
    x = convert_trans_rot_vel_to_steering_angle(linear, angle, WHEEL_DIST)
     
    rospy.loginfo("x : %s, right: %s. left: %s, wheelbase: %s", "/cmd_vel", x, speed_wish_right, speed_wish_left, WHEEL_DIST)
    
    print(x)
    set_speed(2, speed_wish_right)
    set_speed(1, speed_wish_left)
    time.sleep(0.5)
    all_stop()


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)

# def Speed_to_ticks(v):
#   return int(v * cpr / (PID_RATE * PI * wheelDiameter))
#


# initialization
if __name__ == '__main__':

	# setup motor controller
    motor_driver = Adafruit_MotorHAT(i2c_bus=1)

    motor_left_ID = 1
    motor_right_ID = 2
    motor_left = motor_driver.getMotor(motor_left_ID)
    motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
    all_stop()

	# setup ros node
    rospy.init_node('jetbot_motors')
	
    rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
    rospy.Subscriber('~cmd_raw', String, on_cmd_raw)
    rospy.Subscriber('~cmd_str', String, on_cmd_str)
    rospy.Subscriber('/cmd_vel', Twist, vel_callback)
	# start running
    rospy.spin()

	# stop motors before exiting
