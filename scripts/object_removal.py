#!/usr/bin/env python

# ROS Dependencies
import roslib
from simulation_only_msgs.msg import  ObjectRemoval
from std_msgs.msg import Header
import rospy

# Regular Python Dependencies
import time

if __name__ == '__main__':

    rospy.init_node( 'object_initialization' )

    object_id = rospy.get_param("~object_id")
    time_of_removal_walltime = float(rospy.get_param("~time_of_removal_walltime"))
    topic = rospy.get_param("~object_removal_topic")

    publisher = rospy.Publisher( topic, ObjectRemoval, queue_size=6 )

    time.sleep(time_of_removal_walltime)

    obj_removal = ObjectRemoval()
    obj_removal.header = Header()
    obj_removal.object_id = object_id

    publisher.publish(obj_removal)
