#!/usr/bin/env python3

# Import libs
import rospy
from std_msgs.msg import String

def publish():
    # Publisher object, that publish 'String' to topic called 'myTopic'
    p = rospy.Publisher('myTopic', String, queue_size = 10)

    # Initialize the node with the name 'stringPublisher'
    rospy.init_node('stringPublisher')

    # Sleeping rate (10 Hz)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # publish the string 'data' with sleep rate 10 Hz.
        data = "I am the publisher: %s, The time is: %s" % (rospy.get_name(), rospy.get_time())
        p.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass