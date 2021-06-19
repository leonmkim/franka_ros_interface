#!/usr/bin/python

import rospy
import tf.transformations
import numpy as np

# from geometry_msgs.msg import PoseStamped
# from franka_msgs.msg import FrankaState
from franka_core_msgs.msg import EndPointState

from geometry_msgs import Point
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

    for wall in virtual_walls_list:
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

def visualize_walls_callback():
    msg = MarkerArray()
    
    for wall in virtual_walls_list:
        a, b, c, d = wall['a'], wall['b'], wall['c'], wall['d']
        sum = a + b + c

        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0

        # persistent marker only needs to be published once and will not be deleted
        marker.lifetime = 0 

        p_start, p_end = Point()
        p_start.x = a/d
        p_start.y = b/d
        p_start.z = c/d
        marker.points.appent(p_start)
        
        p_end.x = p_start.x + (a/sum)  
        p_end.y = p_start.y + (b/sum)
        p_end.z = p_start.z + (c/sum)
        marker.points.appent(p_end)
        

        msg.markers.append(marker)

    virtual_wall_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("virtual_wall_node")


    # listener = tf.TransformListener()
    # link_name = rospy.get_param("~link_name")

    # get virtual wall constraints from param server
    virtual_walls_list = rospy.get_param('virtual_walls')

    # publish visualization of walls for rviz
    virtual_wall_pub = rospy.Publisher('virtual_wall_viz', MarkerArray)
    rospy.Timer(1, visualize_walls_callback, oneshot=True)

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
