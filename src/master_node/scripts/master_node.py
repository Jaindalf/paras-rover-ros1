#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MasterNode:

    def __init__(self):

        # --- Subscribers ---
        rospy.Subscriber('/toggle', String, self.toggle_callback)
        rospy.Subscriber('/remote', String, self.remote_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # --- Publisher ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # --- State ---
        self.current_mode = "REMOTE"

        rospy.loginfo("Master Node Started. Waiting for data...")

    def toggle_callback(self, msg):

        new_mode = msg.data.strip()

        if self.current_mode != new_mode:
            self.current_mode = new_mode
            rospy.loginfo("Switching Mode to: %s", self.current_mode)
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    # --- Helper: Map Range ---
    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # --- LOGIC 1: MANUAL REMOTE CONTROL ---
    def remote_callback(self, msg):

        if self.current_mode != "REMOTE":
            return

        try:
            parts = msg.data.split(',')

            if len(parts) >= 4:

                raw_ly = int(parts[1])
                raw_rx = int(parts[3])

                twist = Twist()

                if abs(raw_ly - 512) > 30:
                    twist.linear.x = self.map_val(raw_ly, 1023, 0, -1.0, 1.0)
                else:
                    twist.linear.x = 0.0

                if abs(raw_rx - 512) > 30:
                    twist.angular.z = self.map_val(raw_rx, 0, 1023, 1.0, -1.0)
                else:
                    twist.angular.z = 0.0

                self.cmd_vel_pub.publish(twist)

        except ValueError:
            pass

    def scan_callback(self, msg):

        if self.current_mode != "OBSTACLE_AVOIDANCE":
            return

        SAFE_DIST = 1.0
        STOP_DIST = 0.45

        scan_len = len(msg.ranges)

        if scan_len < 100:
            return

        def get_idx(deg):
            return int((deg / 360.0) * scan_len)

        idx_20 = get_idx(20)
        idx_340 = get_idx(340)

        front_ranges = msg.ranges[0:idx_20] + msg.ranges[idx_340:scan_len]

        idx_45 = get_idx(45)
        idx_90 = get_idx(90)
        left_ranges = msg.ranges[idx_45:idx_90]

        idx_270 = get_idx(270)
        idx_315 = get_idx(315)
        right_ranges = msg.ranges[idx_270:idx_315]

        def get_min_dist(scan_slice):
            valid = [r for r in scan_slice if msg.range_min < r < msg.range_max]
            return min(valid) if valid else 10.0

        min_front = get_min_dist(front_ranges)
        min_left = get_min_dist(left_ranges)
        min_right = get_min_dist(right_ranges)

        twist = Twist()

        if min_front < STOP_DIST:

            twist.linear.x = -0.1
            twist.angular.z = 0.0

        elif min_front < SAFE_DIST:

            twist.linear.x = 0.1

            if min_left > min_right:
                twist.angular.z = 1.5
            else:
                twist.angular.z = -1.5

        else:

            twist.linear.x = 0.65
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main():

    rospy.init_node('master_node')

    node = MasterNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()


if __name__ == "__main__":
    main()