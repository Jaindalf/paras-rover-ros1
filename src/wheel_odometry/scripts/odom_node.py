#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf
from tf.transformations import quaternion_from_euler


class WheelOdometry:

    def __init__(self):

        # ---------------- PARAMETERS ----------------
        port = rospy.get_param('~port', '/dev/ttyNano')
        baud = rospy.get_param('~baudrate', 115200)

        # Robot physical parameters
        self.r = 0.06        # wheel radius (meters)
        self.b = 0.63        # wheel separation (meters)
        self.ppr = 268.8     # pulses per revolution

        self.m_per_tick = (2.0 * math.pi * self.r) / self.ppr

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = rospy.Time.now()

        # ---------------- ROS ----------------
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ---------------- SERIAL ----------------
        self.ser = serial.Serial(port, baud, timeout=1)

        # 20 Hz update
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)

        rospy.loginfo("Wheel odometry node started (SLAM-ready)")

    # -------------------------------------------------

    def update(self, event):

        if self.ser.in_waiting == 0:
            return

        line = self.ser.readline().decode(errors='ignore').strip()

        try:
            dL_ticks, dR_ticks = map(float, line.split())
        except:
            return

        # ----------- TIME -----------
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0.0:
            return

        # ----------- TICKS → METERS -----------
        dL = dL_ticks * self.m_per_tick
        dR = dR_ticks * self.m_per_tick

        # ----------- DIFFERENTIAL DRIVE ODOMETRY -----------
        d = (dR + dL) * 0.5
        dtheta = (dR - dL) / self.b

        self.x += d * math.cos(self.theta + 0.5 * dtheta)
        self.y += d * math.sin(self.theta + 0.5 * dtheta)
        self.theta += dtheta

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        vx = d / dt
        vth = dtheta / dt

        self.publish_odom(vx, vth, now)

    # -------------------------------------------------

    def publish_odom(self, vx, vth, stamp):

        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # ---------------- ODOM MESSAGE ----------------
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.pose.covariance = [
            0.05,0,0,0,0,0,
            0,0.05,0,0,0,0,
            0,0,1e6,0,0,0,
            0,0,0,1e6,0,0,
            0,0,0,0,1e6,0,
            0,0,0,0,0,0.1
        ]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        odom.twist.covariance = [
            0.1,0,0,0,0,0,
            0,0.1,0,0,0,0,
            0,0,1e6,0,0,0,
            0,0,0,1e6,0,0,
            0,0,0,0,1e6,0,
            0,0,0,0,0,0.2
        ]

        self.odom_pub.publish(odom)

        # ---------------- TF (odom → base_link) ----------------
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            q,
            stamp,
            "base_link",
            "odom"
        )


# -------------------------------------------------

def main():

    rospy.init_node('wheel_odometry')

    node = WheelOdometry()

    rospy.spin()


if __name__ == '__main__':
    main()