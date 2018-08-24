#!/usr/bin/env python
import sys, os
import time
import math
import copy

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from topp_ros.srv import GetHelixPoints, GetHelixPointsResponse

class HelixPoints():

    def __init__(self):
        # Ros publisher for markers
        self.marker_pub = rospy.Publisher("helix_points", 
            Marker, queue_size=1)
        self.marker = Marker()

        # Create service to receive helical trajectory params
        self.get_helix_points_service = rospy.Service("get_helix_points", 
            GetHelixPoints, self.GetHelixPointsCallback)

    def run(self):
        # Just wait for the request
        rospy.spin()

    def GetHelixPointsCallback(self, req):
        # Setup params as local variables, easier to use that way
        r = req.r
        angleStep = req.angleStep
        x0 = req.x0
        y0 = req.y0
        z0 = req.z0
        zf = req.zf
        deltaZ = req.deltaZ

        # Create waypoint array and pose messages
        res = GetHelixPointsResponse()
        tempPose = Pose()
        n = int((zf-z0)/deltaZ)

        for i in range(n+1):
            current_point = JointTrajectoryPoint()
            x = r*math.cos(float(i)*angleStep) + x0
            y = r*math.sin(float(i)*angleStep) + y0
            z = z0 + float(i)*deltaZ
            yaw = 0.0
            current_point.positions = [x, y, z, yaw]
            tempPose.position.x = x
            tempPose.position.y = y
            tempPose.position.z = z
            tempPose.orientation.w = 1.0
            res.helix_points.points.append(current_point)


        # Create helix for visualization in rviz
        tempMarker = Marker()
        tempMarker.type = Marker.SPHERE_LIST
        tempMarker.action = Marker.ADD
        tempMarker.header.stamp = rospy.Time.now()
        tempMarker.header.frame_id = "map"
        tempMarker.ns = "helix"
        tempMarker.id = 0
        tempMarker.scale.x = 0.07
        tempMarker.scale.y = 0.07
        tempMarker.scale.z = 0.07
        tempMarker.color.r = 1.0
        tempMarker.color.a = 1.0
        tempMarker.lifetime = rospy.Duration()
        tempMarker.pose.orientation.w = 1.0

        tempColor = ColorRGBA()
        tempColor.r = 1.0
        tempColor.a = 1.0

        tempPoint = Point()

        for i in range(n+1):
            tempPoint.x = res.helix_points.points[i].positions[0]
            tempPoint.y = res.helix_points.points[i].positions[1]
            tempPoint.z = res.helix_points.points[i].positions[2]
            tempMarker.points.append(copy.deepcopy(tempPoint))
            tempMarker.colors.append(copy.deepcopy(tempColor))

        self.marker_pub.publish(tempMarker)

        return res


if __name__=="__main__":
    rospy.init_node('get_helix_points_node')
    a = HelixPoints()
    a.run()

# PARAMS for helical trajectory
# r = 1.0
# angle_step = 0.5
# x0 = 0
# y0 = 0
# z0 = 0
# zf = 1.5
# deltaZ = 0.05
