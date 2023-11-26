#pragma once
#include "modern_robotics/modern_robotics.h"

namespace mr
{
    /*
     * Function: Compute s(t) for a cubic time scaling
     * Inputs:
     *  Tf: Total time of the motion in seconds from rest to rest
     *  t: The current time t satisfying 0 < t < Tf
     *
     * Outputs:
     *  st: The path parameter corresponding to a third-order
     *      polynomial motion that begins and ends at zero velocity
     */
    double CubicTimeScaling(double, double);

    /*
     * Function: Compute s(t) for a quintic time scaling
     * Inputs:
     *  Tf: Total time of the motion in seconds from rest to rest
     *  t: The current time t satisfying 0 < t < Tf
     *
     * Outputs:
     *  st: The path parameter corresponding to a fifth-order
     *      polynomial motion that begins and ends at zero velocity
     *	    and zero acceleration
     */
    double QuinticTimeScaling(double, double);

    /*
     * Function: Compute a straight-line trajectory in joint space
     * Inputs:
     *  thetastart: The initial joint variables
     *  thetaend: The final joint variables
     *  Tf: Total time of the motion in seconds from rest to rest
     *	N: The number of points N > 1 (Start and stop) in the discrete
     *     representation of the trajectory
     *  method: The time-scaling method, where 3 indicates cubic (third-
     *          order polynomial) time scaling and 5 indicates quintic
     *          (fifth-order polynomial) time scaling
     *
     * Outputs:
     *  traj: A trajectory as an N x n matrix, where each row is an n-vector
     *        of joint variables at an instant in time. The first row is
     *        thetastart and the Nth row is thetaend . The elapsed time
     *        between each row is Tf / (N - 1)
     */
    Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd&, const Eigen::VectorXd&, double, int, int);

    /*
     * Function: Compute a trajectory as a list of N SE(3) matrices corresponding to
     *			 the screw motion about a space screw axis
     * Inputs:
     *  Xstart: The initial end-effector configuration
     *  Xend: The final end-effector configuration
     *  Tf: Total time of the motion in seconds from rest to rest
     *	N: The number of points N > 1 (Start and stop) in the discrete
     *     representation of the trajectory
     *  method: The time-scaling method, where 3 indicates cubic (third-
     *          order polynomial) time scaling and 5 indicates quintic
     *          (fifth-order polynomial) time scaling
     *
     * Outputs:
     *  traj: The discretized trajectory as a list of N matrices in SE(3)
     *        separated in time by Tf/(N-1). The first in the list is Xstart
     *        and the Nth is Xend
     */
    std::vector<Eigen::MatrixXd> ScrewTrajectory(const Eigen::MatrixXd&, const Eigen::MatrixXd&, double, int, int);

    /*
     * Function: Compute a trajectory as a list of N SE(3) matrices corresponding to
     *			 the origin of the end-effector frame following a straight line
     * Inputs:
     *  Xstart: The initial end-effector configuration
     *  Xend: The final end-effector configuration
     *  Tf: Total time of the motion in seconds from rest to rest
     *	N: The number of points N > 1 (Start and stop) in the discrete
     *     representation of the trajectory
     *  method: The time-scaling method, where 3 indicates cubic (third-
     *          order polynomial) time scaling and 5 indicates quintic
     *          (fifth-order polynomial) time scaling
     *
     * Outputs:
     *  traj: The discretized trajectory as a list of N matrices in SE(3)
     *        separated in time by Tf/(N-1). The first in the list is Xstart
     *        and the Nth is Xend
     * Notes:
     *	This function is similar to ScrewTrajectory, except the origin of the
     *  end-effector frame follows a straight line, decoupled from the rotational
     *  motion.
     */
    std::vector<Eigen::MatrixXd> CartesianTrajectory(const Eigen::MatrixXd&, const Eigen::MatrixXd&, double, int, int);
} // namespace mr
