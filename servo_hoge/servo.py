# import rclpy
# from rclpy.node import Node

# import pigpio

# rclpy.init()
# node = Node("servo")

import time
import pigpio

SERVO_PIN = 23
PULSE_MIN = 500 + 10
PULSE_MAX = 2500 - 10

def angle(theta):
    return (theta / 180) * (PULSE_MAX - PULSE_MIN)

pi = pigpio.pi()

pi.set_servo_pulsewidth(SERVO_PIN, angle(0))

time.sleep(1)

pi.set_servo_pulsewidth(SERVO_PIN, angle(45))
