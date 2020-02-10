#!/usr/bin/env python
#
# Copyright (c) 2017
# FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
# KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# ROS Dependencies
import roslib
from simulation_only_msgs.msg import ObjectRemoval
from std_msgs.msg import Header
import rospy

# Regular Python Dependencies
import time

if __name__ == '__main__':

    rospy.init_node('object_initialization')

    object_id = rospy.get_param("~object_id")
    time_of_removal_walltime = float(rospy.get_param("~time_of_removal_walltime"))
    topic = rospy.get_param("~object_removal_topic")

    publisher = rospy.Publisher(topic, ObjectRemoval, queue_size=6)

    time.sleep(time_of_removal_walltime)

    obj_removal = ObjectRemoval()
    obj_removal.header = Header()
    obj_removal.object_id = object_id

    publisher.publish(obj_removal)
