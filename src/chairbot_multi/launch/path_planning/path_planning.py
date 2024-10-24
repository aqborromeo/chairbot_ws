#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ChairbotPathPlanner:
    def __init__(self):
        # Initialize the node
        rospy.init_node('chairbot_path_planner', anonymous=True)

        # Create a client for move_base action
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Create a publisher for cmd_vel to control the robot's speed
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to the laser scan data for obstacle detection
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Parameters for obstacle avoidance
        self.obstacle_threshold = 0.5  # Minimum distance to consider an obstacle
        self.is_obstacle_ahead = False

    def laser_callback(self, msg):
        # Check for obstacles within a certain distance in the laser scan data
        ranges = msg.ranges
        front_ranges = ranges[len(ranges)//2 - 10: len(ranges)//2 + 10]  # Laser scan in front of the robot
        min_distance = min(front_ranges)

        # Obstacle avoidance logic
        if min_distance < self.obstacle_threshold:
            rospy.loginfo("Obstacle detected! Avoiding obstacle.")
            self.is_obstacle_ahead = True
            self.avoid_obstacle()
        else:
            self.is_obstacle_ahead = False

    def avoid_obstacle(self):
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0.5  # Turn the robot to the left
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

    def send_goal(self, x, y, w):
        # Create a new goal to send to the move_base server
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set goal coordinates (x, y) and orientation (w for quaternion z rotation)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo(f"Sending goal: x={x}, y={y}, w={w}")
        self.client.send_goal(goal)

        # Wait for the result
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available.")
        else:
            rospy.loginfo("Goal reached!")

    def run(self):
        # Define a series of waypoints (x, y, w) the robot should navigate to
        # waypoints = [(2.0, 2.0, 1.0), (0.0, 3.0, 1.0), (-2.0, 0.0, 1.0)]
        # waypoints = [(0.7950, 6.4773, -0.0010), (5.0032, 6.3226, -0.0010)]
        waypoints = [(0.7950, 6.4773, -0.0010), (5.0032, 6.3226, -0.0010), (7.5051, 7.4695, -0.0010)]

        for waypoint in waypoints:
            if not self.is_obstacle_ahead:
                self.send_goal(waypoint[0], waypoint[1], waypoint[2])
                # self.send_goal(waypoint[0], waypoint[1])
                rospy.sleep(2)  # Allow some time between goals
            else:
                rospy.loginfo("Obstacle ahead, waiting for clearance...")
                rospy.sleep(2)  # Wait and keep checking for obstacle avoidance

        rospy.loginfo("Path planning completed.")

if __name__ == '__main__':
    try:
        planner = ChairbotPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
