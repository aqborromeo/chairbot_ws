#!/usr/bin/env python
# license removed for brevity

import rospy
import math
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

global position_tb3_0_x
global position_tb3_0_y
global position_tb3_1_x
global position_tb3_1_y
global position_tb3_2_x
global position_tb3_2_y

waypoints = [
    [(7.5,7.5,0.00),(0, 0, 0, 1),(0)], # one end of table
    [(7.0,6,5.00),(0, 0, 0, 1),(0)],
    [(5.0,6,5.00),(0, 0, 0, 1),(0)],
    [(3.0,6,5.00),(0, 0, 0, 1),(0)],
    [(2.5,7.5,0.00),(0, 0, 0, 1),(0)], # other end of table
    [(3.0,8.5,0.00),(0, 0, 0, 1),(0)],
    [(5.0,8.5,0.00),(0, 0, 0, 1),(0)],
    [(6.4,8.5,0.00),(0, 0, 0, 1),(0)]
    ]

def tb3_0_pos(msg):
  #print("Executing callback0")

  global position_tb3_0_x
  global position_tb3_0_y

  position_tb3_0 = msg.pose.pose.position
  quat_tb3_0 = msg.pose.pose.orientation
  position_tb3_0_x = position_tb3_0.x
  position_tb3_0_y = position_tb3_0.y

def tb3_1_pos(msg):
  #print("Executing callback1")

  global position_tb3_1_x
  global position_tb3_1_y

  position_tb3_1 = msg.pose.pose.position
  quat_tb3_1 = msg.pose.pose.orientation
  position_tb3_1_x = position_tb3_1.x
  position_tb3_1_y = position_tb3_1.y

def tb3_2_pos(msg):
  #print("Executing callback1")

  global position_tb3_2_x
  global position_tb3_2_y

  position_tb3_2 = msg.pose.pose.position
  quat_tb3_2 = msg.pose.pose.orientation
  position_tb3_2_x = position_tb3_2.x
  position_tb3_2_y = position_tb3_2.y

def get_robot_xy(robot):
    if(robot == "tb3_0"):
        return [position_tb3_0_x, position_tb3_0_y]
    elif(robot == "tb3_1"):
        return [position_tb3_1_x, position_tb3_1_y]
    elif(robot == "tb3_2"):
        return [position_tb3_2_x, position_tb3_2_y]
    else:
        return[0.0,0.0]
    

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('tb3_0/move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   
    goal.target_pose.pose.position.x = 3
    goal.target_pose.pose.position.y = 6.03
    goal.target_pose.pose.position.z = 0

    goal.target_pose.pose.orientation.w = 1    

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

def move_to_waypoint(robot):
    client = actionlib.SimpleActionClient(robot+'/move_base',MoveBaseAction) 
    client.wait_for_server()
    robot_pos = get_robot_xy(robot)
    # print(robot_pos)
    min_dist = 10
    chosen_waypoint = int()

    for waypoint in waypoints:
        if(waypoint[2] == 0): # empty waypoint
            x_dist = waypoint[0][0] - robot_pos[0]
            y_dist = waypoint[0][1] - robot_pos[0]
            # print(x_dist,y_dist)
            distance = math.sqrt((x_dist**2)+(y_dist**2))
            # print(distance)
            if(distance < min_dist):
                min_dist = distance
                chosen_waypoint = waypoints.index(waypoint)
    
    print(chosen_waypoint)
    # print(waypoints[chosen_waypoint])
    target_goal = waypoints[chosen_waypoint][:2]
    print(target_goal)
    waypoints[chosen_waypoint][2] = 1
    print(waypoints[chosen_waypoint])
    movebase_multi(robot, target_goal)


def movebase_multi(robot, target):
     # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient(robot+'/move_base',MoveBaseAction) 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    print(target[0])
    goal.target_pose.pose.position.x = target[0][0]
    goal.target_pose.pose.position.y = target[0][1]
    goal.target_pose.pose.position.z = target[0][2]

    print(target[1])
    goal.target_pose.pose.orientation.z = target[1][0]
    goal.target_pose.pose.orientation.y = target[1][1]
    goal.target_pose.pose.orientation.z = target[1][2]
    goal.target_pose.pose.orientation.w = target[1][3]

    # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    targets = [
      [(3.0,6.0,0.00),(0, 0, 0.75, 1)], 
      [(5.0,6.0,0.00),(0, 0, 0.75, 1)], 
      [(7.0,6.0,0.00),(0, 0, 0.75, 1)]
      ]
    
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')

        odom_sub_tb3_0 = rospy.Subscriber('/tb3_0/odom', Odometry, tb3_0_pos)
        odom_sub_tb3_1 = rospy.Subscriber('/tb3_1/odom', Odometry, tb3_1_pos) 
        odom_sub_tb3_2 = rospy.Subscriber('/tb3_2/odom', Odometry, tb3_2_pos) 

        # result = movebase_client()
        result = movebase_multi('tb3_0',targets[0])
        if result:
            rospy.loginfo("tb3_0 Goal execution done!")
        result = movebase_multi('tb3_1',targets[1])
        if result:
            rospy.loginfo("tb3_1 Goal execution done!")
        result = movebase_multi('tb3_2',targets[2])
        if result:
            rospy.loginfo("tb3_2 Goal execution done!")

        # move_to_waypoint('tb3_0')
        # move_to_waypoint('tb3_1')
        # move_to_waypoint('tb3_2')
        
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

