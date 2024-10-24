#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtleBot3PathPlanner:
    def __init__(self, robot_name, waypoints):
        # Store the robot name and unique waypoints
        self.robot_name = robot_name
        self.waypoints = waypoints

        # Initialize the node for this specific robot
        rospy.init_node(f'{self.robot_name}_path_planner', anonymous=True)

        # Action client for move_base in the robot's namespace
        self.client = actionlib.SimpleActionClient(f'/{self.robot_name}/move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Publisher for the robot's cmd_vel topic in its namespace
        self.cmd_vel_pub = rospy.Publisher(f'/{self.robot_name}/cmd_vel', Twist, queue_size=10)

        # Subscribe to the robot's laser scan topic in its namespace
        self.laser_sub = rospy.Subscriber(f'/{self.robot_name}/scan', LaserScan, self.laser_callback)

        # Parameters for obstacle avoidance
        self.obstacle_threshold = 0.5  # Minimum distance to consider an obstacle
        self.is_obstacle_ahead = False

    def laser_callback(self, msg):
        # Process laser scan data to detect obstacles
        ranges = msg.ranges
        front_ranges = ranges[len(ranges)//2 - 10: len(ranges)//2 + 10]  # Laser scan in front of the robot
        min_distance = min(front_ranges)

        # Obstacle avoidance logic
        if min_distance < self.obstacle_threshold:
            rospy.loginfo(f"Obstacle detected by {self.robot_name}! Avoiding obstacle.")
            self.is_obstacle_ahead = True
            self.avoid_obstacle()
        else:
            self.is_obstacle_ahead = False

    def avoid_obstacle(self):
        # Stop and turn the robot to avoid an obstacle
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0.5  # Turn left to avoid
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

    def send_goal(self, x, y, w):
        # Create a new goal for move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set goal coordinates (x, y) and orientation (w)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo(f"{self.robot_name} is sending goal: x={x}, y={y}, w={w}")
        self.client.send_goal(goal)

        # Wait for the result
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr(f"Action server not available for {self.robot_name}!")
            rospy.signal_shutdown(f"Action server not available for {self.robot_name}.")
        else:
            rospy.loginfo(f"Goal reached by {self.robot_name}!")

    def run(self):
        # Navigate through the waypoints for this robot
        for waypoint in self.waypoints:
            if not self.is_obstacle_ahead:
                self.send_goal(waypoint[0], waypoint[1], waypoint[2])
                rospy.sleep(2)  # Allow some time between goals
            else:
                rospy.loginfo(f"{self.robot_name} is waiting for obstacle clearance...")
                rospy.sleep(2)  # Wait and check again

        rospy.loginfo(f"Path planning completed for {self.robot_name}.")

if __name__ == '__main__':
    try:
        # Define the robot names and unique waypoints for each robot
        robot_data = {
            'tb3_0': [(2.0, 2.0, 1.0), (0.0, 3.0, 1.0), (-2.0, 0.0, 1.0)],
            'tb3_1': [(1.0, 1.0, 1.0), (1.0, -2.0, 1.0), (-1.0, -3.0, 1.0)],
            'tb3_2': [(1.0, 1.0, 1.0), (1.0, -2.0, 1.0), (-1.0, -3.0, 1.0)]
        }

        # Create a planner for each robot with its own waypoints
        planners = []
        for robot_name, waypoints in robot_data.items():
            planner = TurtleBot3PathPlanner(robot_name, waypoints)
            planners.append(planner)

        # Run the planners concurrently
        for planner in planners:
            planner.run()

    except rospy.ROSInterruptException:
        pass
