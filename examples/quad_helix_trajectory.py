#!/usr/bin/env python
import sys, os
import time
import math
import copy

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Transform, Twist
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from topp_ros.srv import GetHelixPoints, GetHelixPointsResponse, \
    GetHelixPointsRequest, GenerateTrajectory, GenerateTrajectoryRequest, \
    GenerateTrajectoryResponse


class HelicalTrajectory():

    def __init__(self):
        # First set up service
        trajectory_type = rospy.get_param("~trajectory_type", 
            "generate_toppra_trajectory")
        request_trajectory_service = rospy.ServiceProxy(
            trajectory_type, GenerateTrajectory)

        get_helix_points_service = rospy.ServiceProxy(
            "get_helix_points", GetHelixPoints)

        trajectory_pub = rospy.Publisher('multi_dof_trajectory', 
            MultiDOFJointTrajectory, queue_size=1)
        time.sleep(0.5)

        # Set up helical trajectory
        helix_request = GetHelixPointsRequest()
        helix_request.r = 0.5
        helix_request.angleStep = 0.5
        helix_request.x0 = -0.5
        helix_request.y0 = 0.0
        helix_request.z0 = 1.0
        helix_request.zf = 2.5
        helix_request.deltaZ = 0.05

        # Get the points
        helix_response = get_helix_points_service(helix_request)

        # GetHelixPoints just returns the points. The dynamical part must be 
        # provided by the user.
        dof = len(helix_response.helix_points.points[0].positions)
        helix_response.helix_points.points[0].velocities = [2.0]*dof
        helix_response.helix_points.points[0].accelerations = [1.0]*dof

        # Now call the trajectory generation service
        trajectory_request = GenerateTrajectoryRequest()
        trajectory_request.waypoints = helix_response.helix_points
        trajectory_request.sampling_frequency = 100
        trajectory_response = request_trajectory_service(trajectory_request)
        multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(
            trajectory_response.trajectory)
        trajectory_pub.publish(multi_dof_trajectory)

    def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.w = 1.0

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

if __name__ == "__main__":
    rospy.init_node("quad_helix_trajectory_node")
    a = HelicalTrajectory()