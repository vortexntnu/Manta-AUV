#!/usr/bin/env python
# coding: UTF-8

import roslaunch
import rospy
from std_srvs.srv import Empty

while True:
    key = raw_input()
    if key.lower() == "r":
        rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ingar/development/src/Vortex-Simulator/simulator_launch/launch/quick_reset.launch"])
        launch.start()
        rospy.loginfo("Resetting..")

        rospy.sleep(5)
        # 3 seconds later
        launch.shutdown()
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_simulation()
