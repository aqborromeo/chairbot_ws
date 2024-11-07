#!/usr/bin/env python
import sqlite3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
from datetime import datetime

class WaypointNavigator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waypoint_navigator', anonymous=True)

        # Publisher to send waypoints to the robot
        self.waypoint_pub = rospy.Publisher('/waypoint', PoseWithCovarianceStamped, queue_size=10)

        # Connect to SQLite database
        self.conn = sqlite3.connect('waypoints.db')
        self.cursor = self.conn.cursor()

    def create_events_table(self):
        # Create a new table for storing events
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                datetime TEXT NOT NULL,
                external_id TEXT,
                waypoint_key TEXT NOT NULL,
                status TEXT NOT NULL
            )
        ''')
        self.conn.commit()

    def log_event(self, external_id, waypoint_key, status):
        # Log an event to the events table
        self.cursor.execute('''
            INSERT INTO events (datetime, external_id, waypoint_key, status)
            VALUES (?, ?, ?, ?)
        ''', (datetime.now().isoformat(), external_id, waypoint_key, status))
        self.conn.commit()

    def is_event_progressing(self, external_id):
        # Check if the event is still progressing
        self.cursor.execute('''
            SELECT status FROM events WHERE external_id = ? AND (status = 'PROGRESSING' OR status = 'DONE')
        ''', (external_id,))
        result = self.cursor.fetchone()
        return result is not None

    def navigate_to_waypoints(self, waypoint_key, external_id):
        # Check if the event is still progressing
        if external_id and self.is_event_progressing(external_id):
            rospy.loginfo(f"Event with external_id {external_id} is still progressing. Navigation aborted.")
            return  # Abort navigation if the event is still progressing

        # Retrieve waypoints from the database
        self.cursor.execute('''
            SELECT x, y, theta FROM waypoints WHERE waypoint_key = ?
        ''', (waypoint_key,))
        waypoints = self.cursor.fetchall()

        for waypoint in waypoints:
            x, y, theta = waypoint
            self.publish_waypoint(x, y, theta)
            # Log the event after publishing the waypoint
            self.log_event(external_id, waypoint_key, "Waypoint published")

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