#!/usr/bin/env python

# Toppra imports
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ToppraTrajectory():

    def __init__(self):
        # Basically we have just one service waiting for request and outputting
        # trajectory
        self.generate_toppra_trajectory_service = rospy.Service(
            'generate_toppra_trajectory', GenerateTrajectory, 
            self.generateToppraTrajectoryCallback)

    def run(self):
        # Nothing special, just waiting for service request
        rospy.spin()

    def generateToppraTrajectoryCallback(self, req):
        print "Generating TOPP-RA trajectory."
        res = GenerateTrajectoryResponse()
        dof = len(req.waypoints.points[0].positions)
        n = len(req.waypoints.points)

        # If there is not enough waypoints to generate a trajectory return false
        if (n <= 1 or dof == 0):
            print "You must provide at least 2 points to generate a valid trajectory."
            res.trajectory.success = False
            return res

        # Generate trajectory.
        # First set up waypoints. We know hom many will be from n and dof.
        way_pts = np.zeros([n, dof])
        # Next fill the waypoints with data from request.
        for i in range(0, n):
            for j in range(0, dof):
                way_pts[i][j] = req.waypoints.points[i].positions[j]

        # Part of TOPP-RA is to generate path(s \in [0,1]) from n waypoints.
        # The algorithm then parametrizes the initial path.
        path = ta.SplineInterpolator(np.linspace(0, 1, n), way_pts)

        # Create velocity and acceleration bounds. Supposing symmetrical bounds around zero.
        vlim_ = np.zeros([dof])
        alim_ = np.zeros([dof])
        for i in range(0, dof):
            vlim_[i] = req.waypoints.points[0].velocities[i]
            alim_[i] = req.waypoints.points[0].accelerations[i]
        vlim = np.vstack((-vlim_, vlim_)).T
        alim = np.vstack((-alim_, alim_)).T
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(
            alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

        # Setup a parametrization instance
        instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')

        # Retime the trajectory, only this step is necessary.
        t0 = time.time()
        jnt_traj, aux_traj = instance.compute_trajectory(0, 0)
        #print("Parameterization time: {:} secs".format(time.time() - t0))

        # Convert to JointTrajectory message
        res.trajectory = self.TOPPRA2JointTrajectory(jnt_traj, req.sampling_frequency)
        res.success = True
        return res

    def TOPPRA2JointTrajectory(self, jnt_traj, f):
        # Sampling frequency is required to get the time samples correctly.
        # The number of points in ts_sample is duration*frequency.
        ts_sample = np.linspace(0, jnt_traj.get_duration(), 
            int(jnt_traj.get_duration()*f))
        # Sampling. This returns a matrix for all DOFs. Accessing specific one is 
        # simple: qs_sample[:, 0]
        qs_sample = jnt_traj.eval(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)

        n = qs_sample.shape[0]
        dof = qs_sample.shape[1]

        # Transform into JointTrajectory
        joint_trajectory = JointTrajectory()
        for i in range(0, n):
            temp_point = JointTrajectoryPoint()

            for j in range(0, dof):
                temp_point.positions.append(qs_sample[i,j])
                temp_point.velocities.append(qs_sample[i,j])
                temp_point.accelerations.append(qs_sample[i,j])

            temp_point.time_from_start = rospy.Duration.from_sec(i/f)
            joint_trajectory.points.append(temp_point)

        return joint_trajectory

if __name__ == "__main__":
    rospy.init_node("generate_toppra_trajectory")
    generator = ToppraTrajectory()
    generator.run()