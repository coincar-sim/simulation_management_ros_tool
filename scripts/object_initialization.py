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
from automated_driving_msgs.msg import ObjectStateArray, MotionState, ObjectState, DeltaPoseWithDeltaTime, ClassWithProbability, ObjectClassification
from simulation_only_msgs.msg import ObjectInitialization, DeltaTrajectoryWithID, ObjectRole
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from shape_msgs.msg import Mesh
from shape_msgs.msg import MeshTriangle

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs

import lanelet2
import lanelet2_interface_ros

# Regular Python Dependencies
import time
import xml.etree.ElementTree
import numpy
from math import cos, sin, atan2

x_list = []
y_list = []
d_x_list = []
d_y_list = []
d_t_list = []
x_start = None
y_start = None
velocity = None
object_id = None
cs = None


def import_hull(xml_file):
    e = xml.etree.ElementTree.parse(xml_file).getroot()

    mesh = Mesh()

    for vertice_entry in e.findall('vertice'):
        vertice = Point()
        vertice.x = float(vertice_entry.get('x'))
        vertice.y = float(vertice_entry.get('y'))
        vertice.z = float(vertice_entry.get('z'))
        mesh.vertices.append(vertice)

    for triangle_entry in e.findall('triangle'):
        triangle = MeshTriangle()
        triangle.vertex_indices[0] = int(triangle_entry.get('id0'))
        triangle.vertex_indices[1] = int(triangle_entry.get('id1'))
        triangle.vertex_indices[2] = int(triangle_entry.get('id2'))
        mesh.triangles.append(triangle)

    return mesh


def import_path(xml_file, geoCoordinateProjector):
    x_list = list()
    y_list = list()

    path_osm = lanelet2.io.load(xml_file, geoCoordinateProjector)

    if len(path_osm.lineStringLayer) != 1:
        rospy.logerr("Only one way is allowed in path osm file! "
                     + str(len(path_osm.lineStringLayer)) + " found in file "
                     + xml_file + ". Shutting down!")
        exit()

    for way in path_osm.lineStringLayer:
        for xy_point in way:
            x_list.append(xy_point.x)
            y_list.append(xy_point.y)

    return x_list, y_list


def set_start_and_delta_path(s_start_on_path, velocity, x_list, y_list):
    global d_x_list, d_y_list, d_t_list, x_start, y_start

    i_start = 0
    scale = 0.0

    if s_start_on_path > 0.001:
        found = False
        d_s_list_ = [0.0]
        for i in range(len(x_list)):
            if i > 0:
                d_s = numpy.sqrt((x_list[i] - x_list[i - 1])**2 + (y_list[i] - y_list[i - 1])**2)
                d_s_list_.append(d_s_list_[-1] + d_s)
                if d_s_list_[-1] > s_start_on_path:
                    i_start = i - 1
                    scale = (s_start_on_path - d_s_list_[-2]) / (d_s_list_[-1] - d_s_list_[-2])
                    found = True
                    break
        if not found:
            rospy.logerr("Could not find initial position, is the given " +
                         "s_start_on_path (" + str(s_start_on_path) + ")" +
                         " longer than the path? ")

    x_start = x_list[i_start] + scale * (x_list[i_start + 1] - x_list[i_start])
    y_start = y_list[i_start] + scale * (y_list[i_start + 1] - y_list[i_start])
    d_x_list = [0.0]
    d_y_list = [0.0]
    d_t_list = [0.0]

    dx_second = x_list[i_start + 1] - x_start
    dy_second = y_list[i_start + 1] - y_start
    d_x_list.append(dx_second)
    d_y_list.append(dy_second)
    if velocity < 0.001:
        dt_second = 600.  # 10 minutes
        d_t_list.append(dt_second)
        return
    dt_second = numpy.sqrt(dx_second**2 + dy_second**2) / velocity
    d_t_list.append(dt_second)

    for i in range(len(x_list)):
        if i > i_start + 1:
            dx_total = x_list[i] - x_start
            dy_total = y_list[i] - y_start
            dx_relative = x_list[i] - x_list[i - 1]
            dy_relative = y_list[i] - y_list[i - 1]
            dt = (numpy.sqrt(dx_relative**2 + dy_relative**2) / velocity) + d_t_list[-1]
            d_x_list.append(dx_total)
            d_y_list.append(dy_total)
            d_t_list.append(dt)


def orientation_from_yaw(yaw):
    orientation = Quaternion()
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    orientation.x = quat[0]
    orientation.y = quat[1]
    orientation.z = quat[2]
    orientation.w = quat[3]
    return orientation


def position_from_x_y(x, y):
    position = Point()
    position.x = x
    position.y = y
    position.z = 0.0
    return position


def lanelet_id_from_param_server(param_key):
    '''
    See rosinterface_handler/doc/HowToUseLongParameters.md
    todo: use rosinterface_handler here!
    '''
    assert(param_key, str)
    if not rospy.has_param(param_key):
        rospy.logerr("Cannot retreive \"" + "\" from param server. Shutting down!")
        exit()
    try:
        lanelet_id = int(rospy.get_param(param_key))
        return lanelet_id
    except BaseException:
        try:
            lanelet_id_string_with_long = str(rospy.get_param(param_key))
            lanelet_id_string_with_long = lanelet_id_string_with_long.replace("L", "")
            lanelet_id = int(lanelet_id_string_with_long)
            return lanelet_id
        except BaseException:
            rospy.logerr(
                "Error reading " +
                param_key +
                " from parameter server. See rosinterface_handler/doc/HowToUseLongParameters.md. Value is \"" +
                lanelet_id_string_with_long +
                "\". Shutting down!")
            exit()


if __name__ == '__main__':

    rospy.init_node('object_initialization')

    init_from_lanelets = False
    lanelet_id_start = None
    lanelet_id_goal = None
    if rospy.has_param("~lanelet_id_start") or rospy.has_param("~lanelet_id_goal"):
        lanelet_id_start = lanelet_id_from_param_server("~lanelet_id_start")
        lanelet_id_goal = lanelet_id_from_param_server("~lanelet_id_goal")
        init_from_lanelets = True

    if init_from_lanelets and rospy.has_param("~trajectory_file"):
        rospy.logerr("Initialization works either with lanelets or with trajectory, not both. Shutting down!")
        exit()

    ll2if = lanelet2_interface_ros.Lanelet2InterfaceRos()
    frame_id_initial_position = ll2if.waitForFrameIdMap(10., 10.)

    object_id = rospy.get_param("~object_id")
    velocity = rospy.get_param("~initial_v")
    s_start = rospy.get_param("~s_start", 0.0)
    frame_id_loc_mgmt = rospy.get_param("~frame_id_loc_mgmt")
    topic = rospy.get_param("~object_initialization_topic")

    object_type_name = rospy.get_param("~object_type")
    object_type_id = ObjectClassification.UNCLASSIFIED
    if object_type_name == "car":
        object_type_id = ObjectClassification.CAR
    elif object_type_name == "pedestrian":
        object_type_id = ObjectClassification.PEDESTRIAN
    elif object_type_name == "bicycle":
        object_type_id = ObjectClassification.BICYCLE
    elif object_type_name == "motorbike":
        object_type_id = ObjectClassification.MOTORBIKE
    elif object_type_name == "truck":
        object_type_id = ObjectClassification.TRUCK
    else:
        rospy.logwarn(
            "Object Type \"%s\" not supported; currently supported: " % object_type_name +
            "\"car\", \"pedestrian\", \"truck\",\"motorbike\", \"bicycle\"; classifying object as \"unclassified\"")
    cwp = ClassWithProbability()
    cwp.classification = object_type_id
    cwp.probability = 1.
    object_classification = ObjectClassification()
    object_classification.classes_with_probabilities.append(cwp)

    object_role_name = rospy.get_param("~object_role")
    OBSTACLE_STATIC = 10,
    OBSTACLE_DYNAMIC = 20,
    AGENT_OPERATED = 100
    if object_role_name == "OBSTACLE_STATIC":
        object_role_id = ObjectRole.OBSTACLE_STATIC
    elif object_role_name == "OBSTACLE_DYNAMIC":
        object_role_id = ObjectRole.OBSTACLE_DYNAMIC
    elif object_role_name == "AGENT_OPERATED":
        object_role_id = ObjectRole.AGENT_OPERATED
    else:
        rospy.logwarn(
            "Object Type \"%s\" not known; currently known: \"OBSTACLE_STATIC\", \"OBSTACLE_DYNAMIC\", \"AGENT_OPERATED\"" %
            object_type_name)
    object_role = ObjectRole()
    object_role.type = object_role_id

    publisher = rospy.Publisher(topic, ObjectInitialization, queue_size=6, latch=True)

    x_list = None
    y_list = None
    if init_from_lanelets:
        ll_map = ll2if.waitForNonConstMapPtr(10., 30.)  # pull with 10Hz for max. 10s
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(ll_map, traffic_rules)
        route = None
        x_list = list()
        y_list = list()
        try:
            start_lanelet = ll_map.laneletLayer[lanelet_id_start]
            goal_lanelet = ll_map.laneletLayer[lanelet_id_goal]
            route = graph.getRoute(start_lanelet, goal_lanelet, 0, False)  # No lane changes
        except Exception as e:
            rospy.logwarn("Error in finding route: \"" + e.message + "\".")
            route = None
        if route is None:
            rospy.logwarn(
                "No route found from lanelet \"" +
                str(start_lanelet) +
                "\" to \"" +
                str(goal_lanelet) +
                "\". Trying to initialize on the start lanelet only!")
            for pt in start_lanelet.centerline:
                x_list.append(pt.x)
                y_list.append(pt.y)
        else:
            assert(route.numLanes() == 1), "Only routes with exactly one lane (no lane changes) are currently supported."
            path = route.shortestPath()
            lanelet_sequence = path.getRemainingLane(start_lanelet)
            for pt in lanelet_sequence.centerline:
                x_list.append(pt.x)
                y_list.append(pt.y)
    else:
        path_to_trajectory = rospy.get_param("~trajectory_file")
        geoCoordinateProjector = ll2if.waitForProjectorPtr(10., 30.)
        x_list, y_list = import_path(path_to_trajectory, geoCoordinateProjector)

    set_start_and_delta_path(s_start, velocity, x_list, y_list)

    path_to_hull = rospy.get_param("~hull_file")
    hull = import_hull(path_to_hull)

    spawn_time_seconds = rospy.get_param("~spawn_time")
    spawn_time = rospy.Duration(spawn_time_seconds)

    if not frame_id_initial_position == frame_id_loc_mgmt:
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Special treatment for paused start (with rospy.Time.now() being invalid)
    sim_time_info_sent = False
    while not rospy.is_shutdown():
        if rospy.Time.is_zero(rospy.Time.now()):
            if not sim_time_info_sent:
                rospy.loginfo("Waiting for valid simulation time.")
                sim_time_info_sent = True
            time.sleep(0.01)
        else:
            break

    obj_init = ObjectInitialization()
    obj_init.header.stamp = rospy.Time.now()
    obj_init.header.frame_id = frame_id_loc_mgmt
    obj_init.object_id = object_id

    obj_init.hull = hull
    obj_init.classification = object_classification
    obj_init.role = object_role
    obj_init.spawn_time = spawn_time

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id_initial_position
    pose_stamped.pose.position = position_from_x_y(x_start, y_start)
    pose_stamped.pose.orientation = orientation_from_yaw(0.0)

    if not frame_id_initial_position == frame_id_loc_mgmt:
        rospy.logwarn("Transforming initial_pose of object " + str(object_id) +
                      " from frame " + frame_id_initial_position + " to "
                      "frame " + frame_id_loc_mgmt)
        transform = None
        while not rospy.is_shutdown():
            try:
                transform = tf_buffer.lookup_transform(frame_id_loc_mgmt,
                                                       frame_id_initial_position,  # source frame
                                                       rospy.Time(0),  # get the tf at first available time
                                                       rospy.Duration(1.0))  # wait for 1 second
                break
            except tf2_ros.LookupException:
                rospy.logwarn_throttle(3, "Transform from initial_pose of object " + str(object_id) +
                                          " from frame " + frame_id_initial_position + " to "
                                          "frame " + frame_id_loc_mgmt + " not yet available.")

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        pose_stamped = pose_transformed

    obj_init.initial_pose = Pose()
    obj_init.initial_pose.position = pose_stamped.pose.position
    obj_init.initial_pose.orientation = pose_stamped.pose.orientation

    obj_init.initial_delta_trajectory.header.stamp = rospy.Time.now()
    obj_init.initial_delta_trajectory.object_id = object_id

    if velocity < 0.001:
        # if velocity is almost zero, the object is initialized at full stop
        dpwdt_p = DeltaPoseWithDeltaTime()
        dpwdt_p.delta_time = rospy.Duration(d_t_list[0])
        dpwdt_p.delta_pose.position = position_from_x_y(d_x_list[0], d_y_list[0])
        alpha = atan2(d_y_list[1] - d_y_list[0], d_x_list[1] - d_x_list[0])
        dpwdt_p.delta_pose.orientation = orientation_from_yaw(alpha)
        obj_init.initial_delta_trajectory.delta_poses_with_delta_time.append(dpwdt_p)

        dpwdt_n = DeltaPoseWithDeltaTime()
        dpwdt_n.delta_time = rospy.Duration(d_t_list[1])
        dpwdt_n.delta_pose.position = position_from_x_y(d_x_list[0], d_y_list[0])
        alpha = atan2(d_y_list[1] - d_y_list[0], d_x_list[1] - d_x_list[0])
        dpwdt_n.delta_pose.orientation = orientation_from_yaw(alpha)
        obj_init.initial_delta_trajectory.delta_poses_with_delta_time.append(dpwdt_n)

    else:
        for i in range(len(d_x_list)):
            # delta_pose with orientation of previous section
            if i > 0:
                dpwdt_p = DeltaPoseWithDeltaTime()
                dpwdt_p.delta_time = rospy.Duration(d_t_list[i])
                dpwdt_p.delta_pose.position = position_from_x_y(d_x_list[i], d_y_list[i])
                dpwdt_p.delta_pose.orientation = \
                    obj_init.initial_delta_trajectory.delta_poses_with_delta_time[-1].delta_pose.orientation
                obj_init.initial_delta_trajectory.delta_poses_with_delta_time.append(dpwdt_p)

            # delta_pose with orientation of next section
            if i < len(d_x_list) - 1:
                dpwdt_n = DeltaPoseWithDeltaTime()
                dpwdt_n.delta_time = rospy.Duration(d_t_list[i])
                dpwdt_n.delta_pose.position = position_from_x_y(d_x_list[i], d_y_list[i])
                alpha = atan2(d_y_list[i + 1] - d_y_list[i], d_x_list[i + 1] - d_x_list[i])
                dpwdt_n.delta_pose.orientation = orientation_from_yaw(alpha)
                obj_init.initial_delta_trajectory.delta_poses_with_delta_time.append(dpwdt_n)

    publisher.publish(obj_init)
    rospy.spin()
