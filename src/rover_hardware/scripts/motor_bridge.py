#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time


class MotorBridge:

    def __init__(self):

        # --- Parameters ---
        port = rospy.get_param('~serial_port', '/dev/ttyMega')
        baud = rospy.get_param('~baud_rate', 115200)

        # --- Serial Setup ---
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(3.0)
            rospy.loginfo("Connected to Mega on %s", port)
        except serial.SerialException as e:
            rospy.logerr("Failed to connect to Mega: %s", str(e))
            self.ser = None

        # --- Subscribers ---
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.remote_sub = rospy.Subscriber('/remote', String, self.remote_callback)

        # --- State Variables ---
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.stepper_cmd = 0
        self.servo_cmd = 1

        # --- Timer (20Hz) ---
        rospy.Timer(rospy.Duration(0.05), self.send_serial_command)

    def cmd_callback(self, msg):

        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - angular
        right = linear + angular

        self.left_speed = max(min(left, 1.0), -1.0)
        self.right_speed = max(min(right, 1.0), -1.0)

    def remote_callback(self, msg):

        try:
            parts = msg.data.split(',')

            if len(parts) >= 6:

                raw_stepper_axis = int(parts[2])

                if raw_stepper_axis > 800:
                    self.stepper_cmd = 1
                elif raw_stepper_axis < 200:
                    self.stepper_cmd = -1
                else:
                    self.stepper_cmd = 0

                self.servo_cmd = int(parts[4])

        except ValueError:
            pass

    def send_serial_command(self, event):

        if self.ser:

            command = "{:.2f} {:.2f} {} {}\n".format(
                self.left_speed,
                self.right_speed,
                self.stepper_cmd,
                self.servo_cmd
            )

            try:
                self.ser.write(command.encode())
            except Exception as e:
                rospy.logwarn("Serial Write Error: %s", str(e))


def main():

    rospy.init_node('motor_bridge')

    MotorBridge()

    rospy.spin()


if __name__ == '__main__':
    main()