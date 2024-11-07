#!/usr/bin/env python
import sqlite3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys

class WaypointNavigator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waypoint_navigator', anonymous=True)

        # Publisher to send waypoints to the robot
        self.waypoint_pub = rospy.Publisher('/waypoint', PoseWithCovarianceStamped, queue_size=10)

        # Connect to SQLite database
        self.conn = sqlite3.connect('waypoints.db')
        self.cursor = self.conn.cursor()

    def navigate_to_waypoints(self, waypoint_key):
        # Retrieve waypoints from the database
        self.cursor.execute('''
            SELECT x, y, theta FROM waypoints WHERE waypoint_key = ?
        ''', (waypoint_key,))
        waypoints = self.cursor.fetchall()

        for waypoint in waypoints:
            x, y, theta = waypoint
            self.publish_waypoint(x, y, theta)

    def publish_waypoint(self, x, y, theta):
        # Create a PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = theta  # Assuming theta is represented by z orientation

        # Publish the waypoint
        self.waypoint_pub.publish(msg)
        rospy.sleep(1)  # Sleep for a moment to allow the message to be processed

    def run(self, waypoint_key):
        self.navigate_to_waypoints(waypoint_key)
        rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: rosrun chairbot_routines waypoint_navigator.py <waypoint_key>")
        sys.exit(1)

    waypoint_key = sys.argv[1]
    navigator = WaypointNavigator()
    navigator.run(waypoint_key)