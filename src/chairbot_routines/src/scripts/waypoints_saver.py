#!/usr/bin/env python
import sqlite3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class WaypointSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('waypoint_saver', anonymous=True)

        # Connect to SQLite database
        self.conn = sqlite3.connect('waypoints.db')
        self.cursor = self.conn.cursor()

        # Create table if it doesn't exist
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                waypoint_name TEXT,
                waypoint_key TEXT,
                x REAL,
                y REAL,
                theta REAL,
                map_name TEXT
            )
        ''')
        self.conn.commit()

        # Subscribe to a topic that provides waypoints
        rospy.Subscriber('/waypoint', PoseWithCovarianceStamped, self.waypoint_callback)

    def waypoint_callback(self, msg):
        # Extract waypoint data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z  # Assuming theta is represented by z orientation
        map_name = "office_tables_2"  # Replace with actual map name
        waypoint_name = "Conference room"  # Replace with actual waypoint name
        waypoint_key = "conference_room"  # Replace with actual waypoint key

        # Insert waypoint into the database
        self.cursor.execute('''
            INSERT INTO waypoints (x, y, theta, map_name, waypoint_name, waypoint_key)
            VALUES (?, ?, ?, ?, ?, ?)
        ''', (x, y, theta, map_name, waypoint_name, waypoint_key))
        self.conn.commit()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    waypoint_saver = WaypointSaver()
    waypoint_saver.run()