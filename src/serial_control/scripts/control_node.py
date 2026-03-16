#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import time


class ControlNode:

    def __init__(self):

        # --- Publishers ---
        self.toggle_pub = rospy.Publisher('/toggle', String, queue_size=10)
        self.remote_pub = rospy.Publisher('/remote', String, queue_size=10)

        # --- Serial Connection ---
        try:
            self.ser = serial.Serial('/dev/ttyNano', 115200, timeout=0.1)
            self.ser.flush()
            time.sleep(2)
            rospy.loginfo("Connected to Arduino Nano")
        except serial.SerialException as e:
            rospy.logerr("Could not open serial port: %s", str(e))
            self.ser = None

        # --- Timer (50 Hz) ---
        rospy.Timer(rospy.Duration(0.02), self.timer_callback)

    def timer_callback(self, event):

        if not self.ser:
            return

        if self.ser.in_waiting > 0:
            try:
                data_block = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                lines = data_block.split('\n')

                last_good_line = None

                for line in reversed(lines):
                    line = line.strip()
                    if len(line) > 5 and line.count(',') == 6:
                        last_good_line = line
                        break

                if last_good_line:
                    parts = last_good_line.split(',')

                    # /toggle topic
                    switch_state = int(parts[6])
                    toggle_msg = String()
                    toggle_msg.data = "OBSTACLE_AVOIDANCE" if switch_state == 1 else "REMOTE"
                    self.toggle_pub.publish(toggle_msg)

                    # /remote topic
                    remote_msg = String()
                    remote_msg.data = ",".join(parts[0:6])
                    self.remote_pub.publish(remote_msg)

            except Exception as e:
                rospy.logdebug("Serial skip: %s", str(e))


def main():

    rospy.init_node('control_node')

    node = ControlNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()


if __name__ == '__main__':
    main()