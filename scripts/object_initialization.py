#!/usr/bin/env python

# ROS Dependencies
import roslib
from automated_driving_msgs.msg import ObjectStateArray, MotionState, ObjectState, DeltaPoseWithDeltaTime
from simulation_only_msgs.msg import  ObjectInitialization, DeltaTrajectoryWithID
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

# Regular Python Dependencies
import time
import xml.etree.ElementTree
import pyproj
import numpy
from math import cos, sin, atan2

lat_list = []
lon_list = []
x_list = []
y_list = []
d_x_list = []
d_y_list = []
d_t_list = []
x_start = None
y_start = None
velocity = None
object_id = None

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


def import_path(xml_file):
    e = xml.etree.ElementTree.parse(xml_file).getroot()
    global lat_list, lon_list, x_list, y_list, d_x_list, d_y_list, d_t_list, x_start, y_start, velocity


    for node in e.findall('node'):
        lat_list.append(node.get('lat'))
        lon_list.append(node.get('lon'))
        [x,y] = ll2xy(lat_list[-1],lon_list[-1])
        x_list.append(x)
        y_list.append(y)


def set_start_and_delta_path(s_start_on_path):
    global x_list, y_list, d_x_list, d_y_list, d_t_list, x_start, y_start, velocity

    i_start = 0
    scale = 0.0

    if s_start_on_path > 0.001:
        found = False
        d_s_list_ = [0.0]
        for i in range(len(x_list)):
            if i > 0:
                d_s = numpy.sqrt((x_list[i]-x_list[i-1])**2 + (y_list[i]-y_list[i-1])**2)
                d_s_list_.append(d_s_list_[-1] + d_s)
                if d_s_list_[-1] > s_start_on_path:
                    i_start = i-1
                    scale = (s_start_on_path - d_s_list_[-2]) / (d_s_list_[-1] - d_s_list_[-2])
                    found = True
                    break
        if not found:
            rospy.logerror("Could not find initial position, is the given " + \
                          "s_start_on_path (" + str(s_start_on_path) + ")" + \
                          " longer than the path? ")


    x_start = x_list[i_start] + scale * (x_list[i_start+1] - x_list[i_start])
    y_start = y_list[i_start] + scale * (y_list[i_start+1] - y_list[i_start])
    d_x_list = [0.0]
    d_y_list = [0.0]
    d_t_list = [0.0]

    dx_second = x_list[i_start+1] - x_start
    dy_second = y_list[i_start+1] - y_start
    dt_second = numpy.sqrt(dx_second**2 + dy_second**2) / velocity
    d_x_list.append(dx_second)
    d_y_list.append(dy_second)
    d_t_list.append(dt_second)

    for i in range(len(x_list)):
        if i > i_start+1:
            dx_total = x_list[i] - x_start
            dy_total = y_list[i] - y_start
            dx_relative = x_list[i] - x_list[i-1]
            dy_relative = y_list[i] - y_list[i-1]
            dt = (numpy.sqrt(dx_relative**2 + dy_relative**2) / velocity ) + d_t_list[-1]
            d_x_list.append(dx_total)
            d_y_list.append(dy_total)
            d_t_list.append(dt)


def ll2xy(lat, lon):
    p = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')
    [x, y] = p(lon, lat)
    return [x, y]


def xy2ll(x, y):
    p = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')
    [lon, lat] = p(x, y, inverse=True)
    return [lat, lon]


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

if __name__ == '__main__':

    rospy.init_node( 'object_initialization' )

    object_id = rospy.get_param("~object_id")
    velocity = rospy.get_param("~initial_v")
    s_start = rospy.get_param("~s_start", 0.0)
    frame_id_initial_position = rospy.get_param("~frame_id_initial_position")
    frame_id_loc_mgmt = rospy.get_param("~frame_id_loc_mgmt")
    topic = rospy.get_param("~object_initialization_topic")
    publisher = rospy.Publisher( topic, ObjectInitialization, queue_size=6 )

    path_to_trajectory = rospy.get_param("~trajectory_file")
    import_path(path_to_trajectory)
    set_start_and_delta_path(s_start)

    path_to_hull = rospy.get_param("~hull_file")
    hull = import_hull(path_to_hull)

    if not frame_id_initial_position == frame_id_loc_mgmt:
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

    time.sleep(3)

    obj_init = ObjectInitialization()
    obj_init.header = Header()
    obj_init.header.frame_id = frame_id_loc_mgmt
    obj_init.object_id = object_id

    obj_init.hull = hull

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id_initial_position
    pose_stamped.pose.position = position_from_x_y(x_start, y_start)
    pose_stamped.pose.orientation = orientation_from_yaw(0.0)

    if not frame_id_initial_position == frame_id_loc_mgmt:
        rospy.logwarn("Transforming initial_pose of object " + str(object_id) + \
                      " from frame " + frame_id_initial_position + " to "
                      "frame " + frame_id_loc_mgmt)
        transform = tf_buffer.lookup_transform(frame_id_loc_mgmt,
                                               frame_id_initial_position, #source frame
                                               rospy.Time(0), #get the tf at first available time
                                               rospy.Duration(1.0)) #wait for 1 second
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        pose_stamped = pose_transformed


    obj_init.initial_pose = Pose()
    obj_init.initial_pose.position = pose_stamped.pose.position
    obj_init.initial_pose.orientation = pose_stamped.pose.orientation

    obj_init.initial_delta_trajectory.object_id = object_id

    for i in range(len(d_x_list)):
        # delta_pose with orientation of previous section
        if i>0:
            dpwdt_p = DeltaPoseWithDeltaTime()
            dpwdt_p.delta_time = rospy.Duration(d_t_list[i])
            dpwdt_p.delta_pose.position = position_from_x_y(d_x_list[i], d_y_list[i])
            dpwdt_p.delta_pose.orientation = \
                obj_init.initial_delta_trajectory.delta_poses_with_delta_time[-1].delta_pose.orientation
            obj_init.initial_delta_trajectory.delta_poses_with_delta_time.append(dpwdt_p)

        # delta_pose with orientation of next section
        if i < len(d_x_list)-1:
            dpwdt_n = DeltaPoseWithDeltaTime()
            dpwdt_n.delta_time = rospy.Duration(d_t_list[i])
            dpwdt_n.delta_pose.position = position_from_x_y(d_x_list[i], d_y_list[i])
            alpha = atan2(d_y_list[i+1]-d_y_list[i], d_x_list[i+1]-d_x_list[i])
            dpwdt_n.delta_pose.orientation = orientation_from_yaw(alpha)
            obj_init.initial_delta_trajectory.delta_poses_with_delta_time.append(dpwdt_n)

    publisher.publish(obj_init)