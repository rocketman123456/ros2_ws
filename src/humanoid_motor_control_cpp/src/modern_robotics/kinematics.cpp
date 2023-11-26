#include "modern_robotics/kinematics.h"

namespace mr
{
    /* Function: Compute end effector frame (used for current spatial position calculation)
     * Inputs: Home configuration (position and orientation) of end-effector
     *		   The joint screw axes in the space frame when the manipulator
     *             is at the home position
     * 		   A list of joint coordinates.
     * Returns: Transfomation matrix representing the end-effector frame when the joints are
     *				at the specified coordinates
     * Notes: FK means Forward Kinematics
     */
    Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList)
    {
        Eigen::MatrixXd T = M;
        for (int i = (thetaList.size() - 1); i > -1; i--)
        {
            T = MatrixExp6(VecTose3(Slist.col(i) * thetaList(i))) * T;
        }
        return T;
    }

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
    Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList)
    {
        Eigen::MatrixXd T = M;
        for (int i = 0; i < thetaList.size(); i++)
        {
            T = T * MatrixExp6(VecTose3(Blist.col(i) * thetaList(i)));
        }
        return T;
    }

    /* Function: Gives the space Jacobian
     * Inputs: Screw axis in home position, joint configuration
     * Returns: 6xn Spatial Jacobian
     */
    Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList)
    {
        Eigen::MatrixXd Js = Slist;
        Eigen::MatrixXd T  = Eigen::MatrixXd::Identity(4, 4);
        Eigen::VectorXd sListTemp(Slist.col(0).size());
        for (int i = 1; i < thetaList.size(); i++)
        {
            sListTemp << Slist.col(i - 1) * thetaList(i - 1);
            T = T * MatrixExp6(VecTose3(sListTemp));
            // std::cout << "array: " << sListTemp << std::endl;
            Js.col(i) = Adjoint(T) * Slist.col(i);
        }

        return Js;
    }

    /*
     * Function: Gives the body Jacobian
     * Inputs: Screw axis in BODY position, joint configuration
     * Returns: 6xn Bobdy Jacobian
     */
    Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& thetaList)
    {
        Eigen::MatrixXd Jb = Blist;
        Eigen::MatrixXd T  = Eigen::MatrixXd::Identity(4, 4);
        Eigen::VectorXd bListTemp(Blist.col(0).size());
        for (int i = thetaList.size() - 2; i >= 0; i--)
        {
            bListTemp << Blist.col(i + 1) * thetaList(i + 1);
            T = T * MatrixExp6(VecTose3(-1 * bListTemp));
            // std::cout << "array: " << sListTemp << std::endl;
            Jb.col(i) = Adjoint(T) * Blist.col(i);
        }
        return Jb;
    }

    bool IKinBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T, Eigen::VectorXd& thetalist, double eomg, double ev)
    {
        int             i             = 0;
        int             maxiterations = 20;
        Eigen::MatrixXd Tfk           = FKinBody(M, Blist, thetalist);
        Eigen::MatrixXd Tdiff         = TransInv(Tfk) * T;
        Eigen::VectorXd Vb            = se3ToVec(MatrixLog6(Tdiff));
        Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
        Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));

        bool            err = (angular.norm() > eomg || linear.norm() > ev);
        Eigen::MatrixXd Jb;
        while (err && i < maxiterations)
        {
            Jb = JacobianBody(Blist, thetalist);
            thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
            i += 1;
            // iterate
            Tfk     = FKinBody(M, Blist, thetalist);
            Tdiff   = TransInv(Tfk) * T;
            Vb      = se3ToVec(MatrixLog6(Tdiff));
            angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
            linear  = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
            err     = (angular.norm() > eomg || linear.norm() > ev);
        }
        return !err;
    }

    bool IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T, Eigen::VectorXd& thetalist, double eomg, double ev)
    {
        int             i             = 0;
        int             maxiterations = 20;
        Eigen::MatrixXd Tfk           = FKinSpace(M, Slist, thetalist);
        Eigen::MatrixXd Tdiff         = TransInv(Tfk) * T;
        Eigen::VectorXd Vs            = Adjoint(Tfk) * se3ToVec(MatrixLog6(Tdiff));
        Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
        Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

        bool            err = (angular.norm() > eomg || linear.norm() > ev);
        Eigen::MatrixXd Js;
        while (err && i < maxiterations)
        {
            Js = JacobianSpace(Slist, thetalist);
            thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
            i += 1;
            // iterate
            Tfk     = FKinSpace(M, Slist, thetalist);
            Tdiff   = TransInv(Tfk) * T;
            Vs      = Adjoint(Tfk) * se3ToVec(MatrixLog6(Tdiff));
            angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
            linear  = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
            err     = (angular.norm() > eomg || linear.norm() > ev);
        }
        return !err;
    }

    bool IKinBodyPseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double) {}

    bool IKinSpacePseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double) {}

    bool IKinBodyDamped(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double) {}

    bool IKinSpaceDamped(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double) {}

    bool IKinBodyDampedPseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double, double) {}

    bool IKinSpaceDampedPseudoInverse(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double, double, double) {}

    bool IKinBodyDampedLeastSquare(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, const Eigen::MatrixXd&, double, double) {}

    bool IKinSpaceDampedLeastSquare(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, const Eigen::MatrixXd&, double, double) {}
} // namespace me
