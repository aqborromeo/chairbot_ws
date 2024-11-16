#!/usr/bin/env python
import rospy
import xml.etree.ElementTree as ET
import sys

class RobotBaseWidthRetriever:
    def __init__(self, namespace):
        # Initialize the ROS node
        rospy.init_node('robot_base_width_retriever', anonymous=True)

        # Subscribe to the robot_description parameter with the specified namespace
        self.robot_description = rospy.get_param(f'{namespace}/robot_description')
        self.base_width = self.get_base_width()

    def get_base_width(self):
        # Parse the URDF to find the base width
        root = ET.fromstring(self.robot_description)
        # Assuming the base width is defined in the <link> element for the base
        for link in root.findall('link'):
            if link.get('name') == 'base_link':  # Change 'base_link' if your robot uses a different name
                # Extract the dimensions from the <collision> or <visual> element
                for collision in link.findall('collision'):
                    geometry = collision.find('geometry')
                    if geometry is not None:
                        box = geometry.find('box')
                        if box is not None:
                            size = box.find('size')
                            if size is not None:
                                dimensions = size.text.split()
                                # Assuming the width is the first dimension
                                return float(dimensions[0])  # Return the width
        rospy.logerr("Base width not found in robot description.")
        return None

    def run(self):
        rospy.loginfo(f"Robot base width: {self.base_width}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: rosrun chairbot_routines robot_base_width_retriever.py <namespace>")
        sys.exit(1)

    namespace = sys.argv[1]  # Get the namespace from command line argument
    retriever = RobotBaseWidthRetriever(namespace)
    retriever.run()