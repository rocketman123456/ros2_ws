#pragma once
#include "modern_robotics/modern_robotics.h"

namespace mr
{
    /*
     * Function: This function uses forward-backward Newton-Euler iterations to solve the
     * equation:
     * taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
     *           + g(thetalist) + Jtr(thetalist) * Ftip
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  dthetalist: n-vector of joint rates
     *  ddthetalist: n-vector of joint accelerations
     *  g: Gravity vector g
     *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  taulist: The n-vector of required joint forces/torques
     *
     */
    Eigen::VectorXd InverseDynamics(const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const std::vector<Eigen::MatrixXd>&,
                                    const std::vector<Eigen::MatrixXd>&,
                                    const Eigen::MatrixXd&);

    /*
     * Function: This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and
     *   ddthetalist = 0. The purpose is to calculate one important term in the dynamics equation
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  g: Gravity vector g
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  grav: The 3-vector showing the effect force of gravity to the dynamics
     *
     */
    Eigen::VectorXd GravityForces(const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

    /*
     * Function: This function calls InverseDynamics n times, each time passing a
     * ddthetalist vector with a single element equal to one and all other
     * inputs set to zero. Each call of InverseDynamics generates a single
     * column, and these columns are assembled to create the inertia matrix.
     *
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  M: The numerical inertia matrix M(thetalist) of an n-joint serial
     *     chain at the given configuration thetalist.
     */
    Eigen::MatrixXd MassMatrix(const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

    /*
     * Function: This function calls InverseDynamics with g = 0, Ftip = 0, and
     * ddthetalist = 0.
     *
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  dthetalist: A list of joint rates
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
     *     terms for a given thetalist and dthetalist.
     */
    Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

    /*
     * Function: This function calls InverseDynamics with g = 0, dthetalist = 0, and
     * ddthetalist = 0.
     *
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  JTFtip: The joint forces and torques required only to create the
     *     end-effector force Ftip.
     */
    Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

    /*
     * Function: This function computes ddthetalist by solving:
     * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist)
     *                                  - g(thetalist) - Jtr(thetalist) * Ftip
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  dthetalist: n-vector of joint rates
     *  taulist: An n-vector of joint forces/torques
     *  g: Gravity vector g
     *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  ddthetalist: The resulting joint accelerations
     *
     */
    Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const Eigen::VectorXd&,
                                    const std::vector<Eigen::MatrixXd>&,
                                    const std::vector<Eigen::MatrixXd>&,
                                    const Eigen::MatrixXd&);

    /*
     * Function: Compute the joint angles and velocities at the next timestep using
        first order Euler integration
     * Inputs:
     *  thetalist[in]: n-vector of joint variables
     *  dthetalist[in]: n-vector of joint rates
     *	ddthetalist: n-vector of joint accelerations
     *  dt: The timestep delta t
     *
     * Outputs:
     *  thetalist[out]: Vector of joint variables after dt from first order Euler integration
     *  dthetalist[out]: Vector of joint rates after dt from first order Euler integration
     */
    void EulerStep(Eigen::VectorXd&, Eigen::VectorXd&, const Eigen::VectorXd&, double);

    /*
     * Function: Compute the joint forces/torques required to move the serial chain along the given
     *	trajectory using inverse dynamics
     * Inputs:
     *  thetamat: An N x n matrix of robot joint variables (N: no. of trajecoty time step points; n: no. of robot joints
     *  dthetamat: An N x n matrix of robot joint velocities
     *  ddthetamat: An N x n matrix of robot joint accelerations
     *	g: Gravity vector g
     *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
     *			 the user should input a zero matrix)
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *
     * Outputs:
     *  taumat: The N x n matrix of joint forces/torques for the specified trajectory, where each of the N rows is the vector
     *			of joint forces/torques at each time step
     */
    Eigen::MatrixXd InverseDynamicsTrajectory(const Eigen::MatrixXd&,
                                              const Eigen::MatrixXd&,
                                              const Eigen::MatrixXd&,
                                              const Eigen::VectorXd&,
                                              const Eigen::MatrixXd&,
                                              const std::vector<Eigen::MatrixXd>&,
                                              const std::vector<Eigen::MatrixXd>&,
                                              const Eigen::MatrixXd&);

    /*
     * Function: Compute the motion of a serial chain given an open-loop history of joint forces/torques
     * Inputs:
     *  thetalist: n-vector of initial joint variables
     *  dthetalist: n-vector of initial joint rates
     *  taumat: An N x n matrix of joint forces/torques, where each row is is the joint effort at any time step
     *	g: Gravity vector g
     *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
     *			 the user should input a zero matrix)
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *	dt: The timestep between consecutive joint forces/torques
     *	intRes: Integration resolution is the number of times integration (Euler) takes places between each time step.
     *			Must be an integer value greater than or equal to 1
     *
     * Outputs: std::vector of [thetamat, dthetamat]
     *  thetamat: The N x n matrix of joint angles resulting from the specified joint forces/torques
     *  dthetamat: The N x n matrix of joint velocities
     */
    std::vector<Eigen::MatrixXd> ForwardDynamicsTrajectory(const Eigen::VectorXd&,
                                                           const Eigen::VectorXd&,
                                                           const Eigen::MatrixXd&,
                                                           const Eigen::VectorXd&,
                                                           const Eigen::MatrixXd&,
                                                           const std::vector<Eigen::MatrixXd>&,
                                                           const std::vector<Eigen::MatrixXd>&,
                                                           const Eigen::MatrixXd&,
                                                           double,
                                                           int);

    /*
     * Function: Compute the joint control torques at a particular time instant
     * Inputs:
     *  thetalist: n-vector of joint variables
     *  dthetalist: n-vector of joint rates
     *	eint: n-vector of the time-integral of joint errors
     *	g: Gravity vector g
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *  thetalistd: n-vector of reference joint variables
     *  dthetalistd: n-vector of reference joint rates
     *  ddthetalistd: n-vector of reference joint accelerations
     *	Kp: The feedback proportional gain (identical for each joint)
     *	Ki: The feedback integral gain (identical for each joint)
     *	Kd: The feedback derivative gain (identical for each joint)
     *
     * Outputs:
     *  tau_computed: The vector of joint forces/torques computed by the feedback
     *				  linearizing controller at the current instant
     */
    Eigen::VectorXd ComputedTorque(const Eigen::VectorXd&,
                                   const Eigen::VectorXd&,
                                   const Eigen::VectorXd&,
                                   const Eigen::VectorXd&,
                                   const std::vector<Eigen::MatrixXd>&,
                                   const std::vector<Eigen::MatrixXd>&,
                                   const Eigen::MatrixXd&,
                                   const Eigen::VectorXd&,
                                   const Eigen::VectorXd&,
                                   const Eigen::VectorXd&,
                                   double,
                                   double,
                                   double);

    /*
     * Function: Compute the motion of a serial chain given an open-loop history of joint forces/torques
     * Inputs:
     *  thetalist: n-vector of initial joint variables
     *  dthetalist: n-vector of initial joint rates
     *	g: Gravity vector g
     *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
     *			 the user should input a zero matrix)
     *  Mlist: List of link frames {i} relative to {i-1} at the home position
     *  Glist: Spatial inertia matrices Gi of the links
     *  Slist: Screw axes Si of the joints in a space frame, in the format
     *         of a matrix with the screw axes as the columns.
     *  thetamatd: An Nxn matrix of desired joint variables from the reference trajectory
     *  dthetamatd: An Nxn matrix of desired joint velocities
     *  ddthetamatd: An Nxn matrix of desired joint accelerations
     *	gtilde: The gravity vector based on the model of the actual robot (actual values given above)
     *  Mtildelist: The link frame locations based on the model of the actual robot (actual values given above)
     *  Gtildelist: The link spatial inertias based on the model of the actual robot (actual values given above)
     *	Kp: The feedback proportional gain (identical for each joint)
     *	Ki: The feedback integral gain (identical for each joint)
     *	Kd: The feedback derivative gain (identical for each joint)
     *	dt: The timestep between points on the reference trajectory
     *	intRes: Integration resolution is the number of times integration (Euler) takes places between each time step.
     *			Must be an integer value greater than or equal to 1
     *
     * Outputs: std::vector of [taumat, thetamat]
     *  taumat: An Nxn matrix of the controllers commanded joint forces/ torques, where each row of n forces/torques
     *			  corresponds to a single time instant
     *  thetamat: The N x n matrix of actual joint angles
     */
    std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd&,
                                                 const Eigen::VectorXd&,
                                                 const Eigen::VectorXd&,
                                                 const Eigen::MatrixXd&,
                                                 const std::vector<Eigen::MatrixXd>&,
                                                 const std::vector<Eigen::MatrixXd>&,
                                                 const Eigen::MatrixXd&,
                                                 const Eigen::MatrixXd&,
                                                 const Eigen::MatrixXd&,
                                                 const Eigen::MatrixXd&,
                                                 const Eigen::VectorXd&,
                                                 const std::vector<Eigen::MatrixXd>&,
                                                 const std::vector<Eigen::MatrixXd>&,
                                                 double,
                                                 double,
                                                 double,
                                                 double,
                                                 int);
} // namespace mr
