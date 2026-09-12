#!/usr/bin/env python3

import math
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance:

    def __init__(self):
        rospy.init_node("obstacle_avoidance")

        self.cmd_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10
        )

        rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback
        )

        self.stop_distance = 0.5
        self.forward_speed = 0.2
        self.turn_speed = 0.8

        rospy.loginfo("Obstacle Avoidance Started")

    def get_min_distance(self, scan, start_angle, end_angle):

        distances = []

        for i, r in enumerate(scan.ranges):

            angle = scan.angle_min + i * scan.angle_increment

            if start_angle <= angle <= end_angle:

                if math.isfinite(r):
                    distances.append(r)

        if distances:
            return min(distances)

        return float("inf")

    def scan_callback(self, scan):

        # Front: -30° to +30°
        front = self.get_min_distance(
            scan,
            math.radians(-30),
            math.radians(30)
        )

        # Left: +30° to +90°
        left = self.get_min_distance(
            scan,
            math.radians(30),
            math.radians(90)
        )

        # Right: -90° to -30°
        right = self.get_min_distance(
            scan,
            math.radians(-90),
            math.radians(-30)
        )

        cmd = Twist()

        rospy.loginfo_throttle(
            1.0,
            "Front: %.2f  Left: %.2f  Right: %.2f",
            front,
            left,
            right
        )

        if front < self.stop_distance:

            cmd.linear.x = 0.0

            if left > right:
                cmd.angular.z = self.turn_speed
            else:
                cmd.angular.z = -self.turn_speed

        else:

            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":

    try:
        ObstacleAvoidance()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass