#!/usr/bin/env python

import rospy
import math
import threading
import actionlib
import json
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

global position_tb3_0_x, position_tb3_0_y
global position_tb3_1_x, position_tb3_1_y
global position_tb3_2_x, position_tb3_2_y

def get_waypoints():
    with open('src/chairbot_routines/src/data/waypoints.json', 'r') as file:
        data = json.load(file)
        waypoints = [[(point[0], point[1], 0.0), (0, 0, point[2], 1)] for point in data]
        return waypoints

waypoints = get_waypoints()

def tb3_0_pos(msg):
    global position_tb3_0_x, position_tb3_0_y
    position_tb3_0_x, position_tb3_0_y = msg.pose.pose.position.x, msg.pose.pose.position.y

def tb3_1_pos(msg):
    global position_tb3_1_x, position_tb3_1_y
    position_tb3_1_x, position_tb3_1_y = msg.pose.pose.position.x, msg.pose.pose.position.y

def tb3_2_pos(msg):
    global position_tb3_2_x, position_tb3_2_y
    position_tb3_2_x, position_tb3_2_y = msg.pose.pose.position.x, msg.pose.pose.position.y

def get_robot_xy(robot):
    if robot == "tb3_0":
        return [position_tb3_0_x, position_tb3_0_y]
    elif robot == "tb3_1":
        return [position_tb3_1_x, position_tb3_1_y]
    elif robot == "tb3_2":
        return [position_tb3_2_x, position_tb3_2_y]
    return [0.0, 0.0]

def movebase_multi(robot, target):
    client = actionlib.SimpleActionClient(robot + '/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target[0][0]
    goal.target_pose.pose.position.y = target[0][1]
    goal.target_pose.pose.position.z = target[0][2]

    goal.target_pose.pose.orientation.x = target[1][0]
    goal.target_pose.pose.orientation.y = target[1][1]
    goal.target_pose.pose.orientation.z = target[1][2]
    goal.target_pose.pose.orientation.w = target[1][3]

    client.send_goal(goal)
    client.wait_for_result()
    
    if not client.wait_for_result():
        rospy.logerr(f"Action server for {robot} not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo(f"{robot} Goal execution done.")


def move_all_robots_simultaneously():
    targets = waypoints

    threads = []
    for idx, robot in enumerate(['tb3_0', 'tb3_1', 'tb3_2']):
        thread = threading.Thread(target=movebase_multi, args=(robot, targets[idx]))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')

        odom_sub_tb3_0 = rospy.Subscriber('/tb3_0/odom', Odometry, tb3_0_pos)
        odom_sub_tb3_1 = rospy.Subscriber('/tb3_1/odom', Odometry, tb3_1_pos)
        odom_sub_tb3_2 = rospy.Subscriber('/tb3_2/odom', Odometry, tb3_2_pos)

        move_all_robots_simultaneously()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")