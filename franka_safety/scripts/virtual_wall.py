#!/usr/bin/python

import rospy
import tf.transformations
import numpy as np
import quaternion

# from geometry_msgs.msg import PoseStamped
# from franka_msgs.msg import FrankaState
from franka_core_msgs.msg import EndPointState

from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

from franka_core_msgs.srv import TriggerError

# marker_pose = PoseStamped()
# initial_pose_found = False
# pose_pub = None
# #[[min_x, max_x], [min_y, max_y], [min_z, max_z]]
# position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

# ---------------------------
# Frame notation
# TS = tip state
# O = origin / base frame
# p is translation vector in R^3
# ---------------------------
# O_p_TS = np.empty(3)


# def publisherCallback(msg, link_name):
#     marker_pose.header.frame_id = link_name
#     marker_pose.header.stamp = rospy.Time(0)
#     pose_pub.publish(marker_pose)


def quaternion_from_vectors(vec1, vec2):
    """ Find the quaternion that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A rotation which when applied to vec1, aligns it with vec2.

    Adapted from https://stackoverflow.com/a/59204638
    """

    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    axis = np.cross(a, b)
    angle = np.arccos(np.dot(a, b))

    rot = axis * angle 
    quat = quaternion.from_rotation_vector(rot).normalized()
    return quat


def tip_state_callback(msg):
    # initial_quaternion = \
    #     tf.transformations.quaternion_from_matrix(
    #         np.transpose(np.reshape(msg.O_T_EE,
    #                                 (4, 4))))
    # initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    # marker_pose.pose.orientation.x = initial_quaternion[0]
    # marker_pose.pose.orientation.y = initial_quaternion[1]
    # marker_pose.pose.orientation.z = initial_quaternion[2]
    # marker_pose.pose.orientation.w = initial_quaternion[3]
    # marker_pose.pose.position.x = msg.O_T_EE[12]
    # marker_pose.pose.position.y = msg.O_T_EE[13]
    # marker_pose.pose.position.z = msg.O_T_EE[14]

    # global initial_pose_found
    # initial_pose_found = True

    # cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4, 4, order='F')
# 
    # self._cartesian_pose = {
        # 'position': cart_pose_trans_mat[:3, 3],
        # 'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3, :3]),
        # 'ori_mat': cart_pose_trans_mat[:3,:3]}

    O_p_TS_x = msg.O_T_EE[12] # x
    O_p_TS_y = msg.O_T_EE[13] # y
    O_p_TS_z = msg.O_T_EE[14] # z

    for wall in virtual_walls_list.values():
        # virtual
        a, b, c, d = wall['a'], wall['b'], wall['c'], wall['d']
        if ((a * O_p_TS_x + b * O_p_TS_y + c * O_p_TS_z) <= d):
            
            try:
                resp = trigger_error(True)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

# def franka_state_callback(msg):
#     initial_quaternion = \
#         tf.transformations.quaternion_from_matrix(
#             np.transpose(np.reshape(msg.O_T_EE,
#                                     (4, 4))))
#     initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
#     marker_pose.pose.orientation.x = initial_quaternion[0]
#     marker_pose.pose.orientation.y = initial_quaternion[1]
#     marker_pose.pose.orientation.z = initial_quaternion[2]
#     marker_pose.pose.orientation.w = initial_quaternion[3]
#     marker_pose.pose.position.x = msg.O_T_EE[12]
#     marker_pose.pose.position.y = msg.O_T_EE[13]
#     marker_pose.pose.position.z = msg.O_T_EE[14]
#     global initial_pose_found
#     initial_pose_found = True

# def processFeedback(feedback):
#     if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
#         marker_pose.pose.position.x = max([min([feedback.pose.position.x,
#                                           position_limits[0][1]]),
#                                           position_limits[0][0]])
#         marker_pose.pose.position.y = max([min([feedback.pose.position.y,
#                                           position_limits[1][1]]),
#                                           position_limits[1][0]])
#         marker_pose.pose.position.z = max([min([feedback.pose.position.z,
#                                           position_limits[2][1]]),
#                                           position_limits[2][0]])
#         marker_pose.pose.orientation = feedback.pose.orientation
#     server.applyChanges()

def visualize_walls_callback(event):
    msg = MarkerArray()
    id = 0
    
    for wall in virtual_walls_list.values():
        a, b, c, d = wall['a'], wall['b'], wall['c'], wall['d']


        # create halfspace normal vector in direction of feasibility
        scale = 0.1
        sum = (a + b + c)
        if d == 0:
            norm = 0
        else:
            norm = (a**2 + b**2 + c**2) / d

        arrow = Marker()
        arrow.id = id
        arrow.header.frame_id = "panda_link0"
        arrow.type = arrow.ARROW
        arrow.action = arrow.ADD
        arrow.scale.x = 0.025
        arrow.scale.y = 0.05
        arrow.scale.z = 0.0
        arrow.color.a = 1.0
        arrow.color.r = 1.0

        # persistent marker only needs to be published once and will not be deleted
        arrow.lifetime = rospy.Duration(1) 

        p_start = Point()
        p_end = Point()
        p_start.x = a/norm
        p_start.y = b/norm
        p_start.z = c/norm
        arrow.points.append(p_start)
        
        p_end.x = (p_start.x + (scale*(a/sum)))  
        p_end.y = (p_start.y + (scale*(b/sum)))
        p_end.z = (p_start.z + (scale*(c/sum)))
        arrow.points.append(p_end)

        msg.markers.append(arrow)

        id+=1

        plane = Marker()
        plane.id = id
        plane.header.frame_id = "panda_link0"
        plane.type = plane.CUBE
        plane.action = plane.ADD
        plane.scale.x = 0.001
        plane.scale.y = 1.0
        plane.scale.z = 1.0
        plane.color.a = 0.3
        plane.color.r = 1.0

        plane.pose.position.x = p_start.x 
        plane.pose.position.y = p_start.y
        plane.pose.position.z = p_start.z

        normal_vector = np.asarray([(a/sum), (b/sum), (c/sum)])
        unit_x = np.asarray([1, 0, 0])
        quat_ori = quaternion_from_vectors(unit_x, normal_vector)

        plane.pose.orientation.x = quat_ori.x
        plane.pose.orientation.y = quat_ori.y
        plane.pose.orientation.z = quat_ori.z
        plane.pose.orientation.w = quat_ori.w

        # persistent marker only needs to be published once and will not be deleted
        plane.lifetime = rospy.Duration(1) 

        msg.markers.append(plane)

        id+=1
    


    virtual_wall_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("virtual_wall_node")


    # listener = tf.TransformListener()
    # link_name = rospy.get_param("~link_name")

    # get virtual wall constraints from param server
    virtual_walls_list = rospy.get_param('virtual_walls')
    # print(virtual_walls_list)
    # for wall in virtual_walls_list.values():
    #     print(wall)

    # publish visualization of walls for rviz
    virtual_wall_pub = rospy.Publisher('virtual_wall_viz', MarkerArray, queue_size=1)
    rospy.Timer(rospy.Duration(1), visualize_walls_callback, oneshot=False)

    # setup service client request for trigger_error
    rospy.wait_for_service('/franka_ros_interface/franka_control/trigger_error')
    trigger_error = rospy.ServiceProxy('/franka_ros_interface/franka_control/trigger_error', TriggerError)

    # start subscribing to tip state
    tip_state_sub = rospy.Subscriber("custom_franka_state_controller/tip_state",
                                 EndPointState, tip_state_callback, queue_size=1, tcp_nodelay=True)

    # virtual_walls = []
    # for (wall in virtual_walls_list):
    #     virtual_walls

    # pose_pub = rospy.Publisher(
    #     "equilibrium_pose", PoseStamped, queue_size=10)
    # server = InteractiveMarkerServer("equilibrium_pose_marker")
    # int_marker = InteractiveMarker()
    # int_marker.header.frame_id = link_name
    # int_marker.scale = 0.3
    # int_marker.name = "equilibrium_pose"
    # int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
    #                           "If you move the \nequilibrium "
    #                           "pose the robot will follow it\n"
    #                           "so be aware of potential collisions")
    # int_marker.pose = marker_pose.pose
    # # run pose publisher
    # rospy.Timer(rospy.Duration(0.005),
    #             lambda msg: publisherCallback(msg, link_name))

    # # insert a box
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 1
    # control.orientation.y = 0
    # control.orientation.z = 0
    # control.name = "rotate_x"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)

    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 1
    # control.orientation.y = 0
    # control.orientation.z = 0
    # control.name = "move_x"
    # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    # int_marker.controls.append(control)
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 1
    # control.orientation.z = 0
    # control.name = "rotate_y"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 1
    # control.orientation.z = 0
    # control.name = "move_y"
    # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    # int_marker.controls.append(control)
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 0
    # control.orientation.z = 1
    # control.name = "rotate_z"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 0
    # control.orientation.z = 1
    # control.name = "move_z"
    # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    # int_marker.controls.append(control)
    # server.insert(int_marker, processFeedback)

    # server.applyChanges()

    rospy.spin()
