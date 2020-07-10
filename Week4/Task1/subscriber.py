#!/usr/bin/env python3

# Import lib
import rospy
from std_msgs.msg import String

def callback(data):
    # Read the data and print it in the node log.
    rospy.loginfo(rospy.get_caller_id() + "Data:  %s", data.data)

def subscriber():
    # Initialize the node with the name 'stringPublisher'
    rospy.init_node('stringSubscriber')
    
    # Subscribe to 'myTopic' reading 'String'
    rospy.Subscriber('myTopic', String, callback)

    # Don't stop until node is dead.
    rospy.spin()

if __name__ == '__main__':
    subscriber()