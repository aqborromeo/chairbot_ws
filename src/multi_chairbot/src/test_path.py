#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

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
      [(3.0,6.0,0.00),(0, 0, 0, 1)], 
      [(5.0,6.0,0.00),(0, 0, 0, 1)], 
      [(7.0,6.0,0.00),(0, 0, 0, 1)]
      ]
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
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
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

"""     targets = [
      [(3,6.03,0.00),(0, 0, 0, 1)], 
      [(7,28,6.40,0.00),(0, 0, 0, 1)], 
      [(7.5,8.82,0.97),(0, 0, 0, 1)]
      ] """
