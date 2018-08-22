import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time

ta.setup_logging("INFO")


def main():
    # Setup the waypoints here. Quad without manipulator has 4 degrees of
    # freedom. We will use (x,y,z,yaw) in this example. Each degree of freedom
    # has N waypoints and we have to arrange those as numpy arrays. Let's try
    # a square set of waypoints
    way_pts = np.array([[0, 0, 1, 0], [0.5, 0, 1, 0], [1, 0, 1, 0], 
        [1, 0.5,  1, 0], [1, 1, 1, 0], [0.5, 1, 1, 0], [0, 1, 1, 0], 
        [0, 0.5, 1, 0], [0, 0, 1, 0]])
    # linspace goes from 0 to 1 with 9 samples. This is because we have
    # 9 waypoints.
    path = ta.SplineInterpolator(np.linspace(0, 1, 9), way_pts)

    # Create velocity bounds, then velocity constraint object
    vlim_low = np.array([-2, -2, -2, -1])
    vlim_high = np.array([2, 2, 2, 1])
    vlim = np.vstack((vlim_low, vlim_high)).T
    # Create acceleration bounds, then acceleration constraint object
    alim_low = np.array([-2, -2, -2, -1])
    alim_high = np.array([2, 2, 2, 1])
    alim = np.vstack((alim_low, alim_high)).T
    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(
        alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

    # Setup a parametrization instance
    instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')

    # Retime the trajectory, only this step is necessary.
    t0 = time.time()
    jnt_traj, aux_traj = instance.compute_trajectory(0, 0)
    print("Parameterization time: {:} secs".format(time.time() - t0))
    # Sampling frequency is required to get the time samples correctly.
    # The number of points in ts_sample is duration*frequency.
    sample_freq = 100
    ts_sample = np.linspace(0, jnt_traj.get_duration(), 
        int(jnt_traj.get_duration()*sample_freq))
    # Sampling. This returns a matrix for all DOFs. Accessing specific one is 
    # simple: qs_sample[:, 0]
    qs_sample = jnt_traj.eval(ts_sample)
    qds_sample = jnt_traj.evald(ts_sample)
    qdds_sample = jnt_traj.evaldd(ts_sample)

    plt.plot(ts_sample, qdds_sample)
    plt.xlabel("Time (s)")
    plt.ylabel("Joint acceleration (rad/s^2)")
    plt.show()

    # Compute the feasible sets and the controllable sets for viewing.
    # Note that these steps are not necessary.
    _, sd_vec, _ = instance.compute_parameterization(0, 0)
    X = instance.compute_feasible_sets()
    K = instance.compute_controllable_sets(0, 0)

    X = np.sqrt(X)
    K = np.sqrt(K)

    plt.plot(X[:, 0], c='green', label="Feasible sets")
    plt.plot(X[:, 1], c='green')
    plt.plot(K[:, 0], '--', c='red', label="Controllable sets")
    plt.plot(K[:, 1], '--', c='red')
    plt.plot(sd_vec, label="Velocity profile")
    plt.title("Path-position path-velocity plot")
    plt.xlabel("Path position")
    plt.ylabel("Path velocity square")
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.plot(qs_sample[:,0], qs_sample[:,1])
    plt.show()
    
    plt.plot(ts_sample, qs_sample[:,1])
    plt.show()

if __name__ == '__main__':
    main()
