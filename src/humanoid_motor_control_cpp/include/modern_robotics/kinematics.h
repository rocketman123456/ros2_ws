#pragma once
#include "modern_robotics/modern_robotics.h"

namespace mr
{
    /*
     * Function: Compute end effector frame (used for current spatial position calculation)
     * Inputs: Home configuration (position and orientation) of end-effector
     *		   The joint screw axes in the space frame when the manipulator
     *             is at the home position
     * 		   A list of joint coordinates.
     * Returns: Transfomation matrix representing the end-effector frame when the joints are
     *				at the specified coordinates
     * Notes: FK means Forward Kinematics
     */
    Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::VectorXd&);

    /*
     * Function: Compute end effector frame (used for current body position calculation)
     * Inputs: Home configuration (position and orientation) of end-effector
     *		   The joint screw axes in the body frame when the manipulator
     *             is at the home position
     * 		   A list of joint coordinates.
     * Returns: Transfomation matrix representing the end-effector frame when the joints are
     *				at the specified coordinates
     * Notes: FK means Forward Kinematics
     */
    Eigen::MatrixXd FKinBody(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::VectorXd&);

    /*
     * Function: Gives the space Jacobian
     * Inputs: Screw axis in home position, joint configuration
     * Returns: 6xn Spatial Jacobian
     */
    Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

    /*
     * Function: Gives the body Jacobian
     * Inputs: Screw axis in BODY position, joint configuration
     * Returns: 6xn Bobdy Jacobian
     */
    Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

    /*
     * Function: Computes inverse kinematics in the body frame for an open chain robot
     * Inputs:
     *	Blist: The joint screw axes in the end-effector frame when the
     *         manipulator is at the home position, in the format of a
     *         matrix with axes as the columns
     *	M: The home configuration of the end-effector
     *	T: The desired end-effector configuration Tsd
     *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
     *         satisfying Tsd
     *	emog: A small positive tolerance on the end-effector orientation
     *        error. The returned joint angles must give an end-effector
     *        orientation error less than eomg
     *	ev: A small positive tolerance on the end-effector linear position
     *      error. The returned joint angles must give an end-effector
     *      position error less than ev
     * Outputs:
     *	success: A logical value where TRUE means that the function found
     *           a solution and FALSE means that it ran through the set
     *           number of maximum iterations without finding a solution
     *           within the tolerances eomg and ev.
     *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
     */
    bool IKinBody(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double);

    /*
     * Function: Computes inverse kinematics in the space frame for an open chain robot
     * Inputs:
     *	Slist: The joint screw axes in the space frame when the
     *         manipulator is at the home position, in the format of a
     *         matrix with axes as the columns
     *	M: The home configuration of the end-effector
     *	T: The desired end-effector configuration Tsd
     *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
     *         satisfying Tsd
     *	emog: A small positive tolerance on the end-effector orientation
     *        error. The returned joint angles must give an end-effector
     *        orientation error less than eomg
     *	ev: A small positive tolerance on the end-effector linear position
     *      error. The returned joint angles must give an end-effector
     *      position error less than ev
     * Outputs:
     *	success: A logical value where TRUE means that the function found
     *           a solution and FALSE means that it ran through the set
     *           number of maximum iterations without finding a solution
     *           within the tolerances eomg and ev.
     *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
     */
    bool IKinSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double);

    // TODO : multi ik algorithm
    bool IKinBodyPseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double);
    bool IKinSpacePseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double);
    bool IKinBodyDamped(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double);
    bool IKinSpaceDamped(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double);
    bool IKinBodyDampedPseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double, double);
    bool IKinSpaceDampedPseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double, double);
    bool IKinBodyDampedLeastSquare(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, const Eigen::MatrixXd&, double, double);
    bool IKinSpaceDampedLeastSquare(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, const Eigen::MatrixXd&, double, double);

} // namespace mr
