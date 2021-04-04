#!/usr/bin/env python
# coding: UTF-8

import roslaunch
import rospy

rospy.sleep(0.02)
rospy.init_node('shutdown', anonymous=True)
print("Shutting down nodes...")
