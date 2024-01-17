#!/usr/bin/env python3

import rospy

if __name__ == '__main__':
    rospy.init_node("test_nodes")

    rospy.loginfo("Test node has been initialized!")
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")

        rate.sleep()
    