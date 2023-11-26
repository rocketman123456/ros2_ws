#include "modern_robotics/dynamcics.h"

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
    Eigen::VectorXd InverseDynamics(const Eigen::VectorXd&              thetalist,
                                    const Eigen::VectorXd&              dthetalist,
                                    const Eigen::VectorXd&              ddthetalist,
                                    const Eigen::VectorXd&              g,
                                    const Eigen::VectorXd&              Ftip,
                                    const std::vector<Eigen::MatrixXd>& Mlist,
                                    const std::vector<Eigen::MatrixXd>& Glist,
                                    const Eigen::MatrixXd&              Slist)
    {
        // the size of the lists
        int n = thetalist.size();

        Eigen::MatrixXd              Mi = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd              Ai = Eigen::MatrixXd::Zero(6, n);
        std::vector<Eigen::MatrixXd> AdTi;
        for (int i = 0; i < n + 1; i++)
        {
            AdTi.push_back(Eigen::MatrixXd::Zero(6, 6));
        }
        Eigen::MatrixXd Vi  = Eigen::MatrixXd::Zero(6, n + 1); // velocity
        Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6, n + 1); // acceleration

        Vdi.block(3, 0, 3, 1) = -g;
        AdTi[n]               = mr::Adjoint(mr::TransInv(Mlist[n]));
        Eigen::VectorXd Fi    = Ftip;

        Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);

        // forward pass
        for (int i = 0; i < n; i++)
        {
            Mi        = Mi * Mlist[i];
            Ai.col(i) = mr::Adjoint(mr::TransInv(Mi)) * Slist.col(i);

            AdTi[i] = mr::Adjoint(mr::MatrixExp6(mr::VecTose3(Ai.col(i) * -thetalist(i))) * mr::TransInv(Mlist[i]));

            Vi.col(i + 1)  = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
            Vdi.col(i + 1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i) + ad(Vi.col(i + 1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
        }

        // backward pass
        for (int i = n - 1; i >= 0; i--)
        {
            Fi         = AdTi[i + 1].transpose() * Fi + Glist[i] * Vdi.col(i + 1) - ad(Vi.col(i + 1)).transpose() * (Glist[i] * Vi.col(i + 1));
            taulist(i) = Fi.transpose() * Ai.col(i);
        }
        return taulist;
    }

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
    Eigen::VectorXd
    GravityForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& g, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist)
    {
        int             n          = thetalist.size();
        Eigen::VectorXd dummylist  = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd grav       = mr::InverseDynamics(thetalist, dummylist, dummylist, g, dummyForce, Mlist, Glist, Slist);
        return grav;
    }

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
    Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist)
    {
        int             n          = thetalist.size();
        Eigen::VectorXd dummylist  = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd dummyg     = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd M          = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < n; i++)
        {
            Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
            ddthetalist(i)              = 1;
            M.col(i)                    = mr::InverseDynamics(thetalist, dummylist, ddthetalist, dummyg, dummyforce, Mlist, Glist, Slist);
        }
        return M;
    }

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
    Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd&              thetalist,
                                       const Eigen::VectorXd&              dthetalist,
                                       const std::vector<Eigen::MatrixXd>& Mlist,
                                       const std::vector<Eigen::MatrixXd>& Glist,
                                       const Eigen::MatrixXd&              Slist)
    {
        int             n          = thetalist.size();
        Eigen::VectorXd dummylist  = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd dummyg     = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd c          = mr::InverseDynamics(thetalist, dthetalist, dummylist, dummyg, dummyforce, Mlist, Glist, Slist);
        return c;
    }

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
    Eigen::VectorXd
    EndEffectorForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist)
    {
        int             n         = thetalist.size();
        Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd dummyg    = Eigen::VectorXd::Zero(3);

        Eigen::VectorXd JTFtip = mr::InverseDynamics(thetalist, dummylist, dummylist, dummyg, Ftip, Mlist, Glist, Slist);
        return JTFtip;
    }

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
    Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd&              thetalist,
                                    const Eigen::VectorXd&              dthetalist,
                                    const Eigen::VectorXd&              taulist,
                                    const Eigen::VectorXd&              g,
                                    const Eigen::VectorXd&              Ftip,
                                    const std::vector<Eigen::MatrixXd>& Mlist,
                                    const std::vector<Eigen::MatrixXd>& Glist,
                                    const Eigen::MatrixXd&              Slist)
    {

        Eigen::VectorXd totalForce = taulist - mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist) - mr::GravityForces(thetalist, g, Mlist, Glist, Slist) -
                                     mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

        Eigen::MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);

        // Use LDLT since M is positive definite
        Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);

        return ddthetalist;
    }

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
    void EulerStep(Eigen::VectorXd& thetalist, Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist, double dt)
    {
        thetalist += dthetalist * dt;
        dthetalist += ddthetalist * dt;
        return;
    }

    Eigen::MatrixXd InverseDynamicsTrajectory(const Eigen::MatrixXd&              thetamat,
                                              const Eigen::MatrixXd&              dthetamat,
                                              const Eigen::MatrixXd&              ddthetamat,
                                              const Eigen::VectorXd&              g,
                                              const Eigen::MatrixXd&              Ftipmat,
                                              const std::vector<Eigen::MatrixXd>& Mlist,
                                              const std::vector<Eigen::MatrixXd>& Glist,
                                              const Eigen::MatrixXd&              Slist)
    {
        Eigen::MatrixXd thetamatT   = thetamat.transpose();
        Eigen::MatrixXd dthetamatT  = dthetamat.transpose();
        Eigen::MatrixXd ddthetamatT = ddthetamat.transpose();
        Eigen::MatrixXd FtipmatT    = Ftipmat.transpose();

        int             N       = thetamat.rows(); // trajectory points
        int             dof     = thetamat.cols();
        Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(dof, N);
        for (int i = 0; i < N; ++i)
        {
            taumatT.col(i) = InverseDynamics(thetamatT.col(i), dthetamatT.col(i), ddthetamatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
        }
        Eigen::MatrixXd taumat = taumatT.transpose();
        return taumat;
    }

    std::vector<Eigen::MatrixXd> ForwardDynamicsTrajectory(const Eigen::VectorXd&              thetalist,
                                                           const Eigen::VectorXd&              dthetalist,
                                                           const Eigen::MatrixXd&              taumat,
                                                           const Eigen::VectorXd&              g,
                                                           const Eigen::MatrixXd&              Ftipmat,
                                                           const std::vector<Eigen::MatrixXd>& Mlist,
                                                           const std::vector<Eigen::MatrixXd>& Glist,
                                                           const Eigen::MatrixXd&              Slist,
                                                           double                              dt,
                                                           int                                 intRes)
    {
        Eigen::MatrixXd taumatT       = taumat.transpose();
        Eigen::MatrixXd FtipmatT      = Ftipmat.transpose();
        int             N             = taumat.rows(); // force/torque points
        int             dof           = taumat.cols();
        Eigen::MatrixXd thetamatT     = Eigen::MatrixXd::Zero(dof, N);
        Eigen::MatrixXd dthetamatT    = Eigen::MatrixXd::Zero(dof, N);
        thetamatT.col(0)              = thetalist;
        dthetamatT.col(0)             = dthetalist;
        Eigen::VectorXd thetacurrent  = thetalist;
        Eigen::VectorXd dthetacurrent = dthetalist;
        Eigen::VectorXd ddthetalist;
        for (int i = 0; i < N - 1; ++i)
        {
            for (int j = 0; j < intRes; ++j)
            {
                ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taumatT.col(i), g, FtipmatT.col(i), Mlist, Glist, Slist);
                EulerStep(thetacurrent, dthetacurrent, ddthetalist, 1.0 * dt / intRes);
            }
            thetamatT.col(i + 1)  = thetacurrent;
            dthetamatT.col(i + 1) = dthetacurrent;
        }
        std::vector<Eigen::MatrixXd> JointTraj_ret;
        JointTraj_ret.push_back(thetamatT.transpose());
        JointTraj_ret.push_back(dthetamatT.transpose());
        return JointTraj_ret;
    }

    Eigen::VectorXd ComputedTorque(const Eigen::VectorXd&              thetalist,
                                   const Eigen::VectorXd&              dthetalist,
                                   const Eigen::VectorXd&              eint,
                                   const Eigen::VectorXd&              g,
                                   const std::vector<Eigen::MatrixXd>& Mlist,
                                   const std::vector<Eigen::MatrixXd>& Glist,
                                   const Eigen::MatrixXd&              Slist,
                                   const Eigen::VectorXd&              thetalistd,
                                   const Eigen::VectorXd&              dthetalistd,
                                   const Eigen::VectorXd&              ddthetalistd,
                                   double                              Kp,
                                   double                              Ki,
                                   double                              Kd)
    {

        Eigen::VectorXd e               = thetalistd - thetalist; // position err
        Eigen::VectorXd tau_feedforward = MassMatrix(thetalist, Mlist, Glist, Slist) * (Kp * e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));

        Eigen::VectorXd Ftip           = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd tau_inversedyn = InverseDynamics(thetalist, dthetalist, ddthetalistd, g, Ftip, Mlist, Glist, Slist);

        Eigen::VectorXd tau_computed = tau_feedforward + tau_inversedyn;
        return tau_computed;
    }

    std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd&              thetalist,
                                                 const Eigen::VectorXd&              dthetalist,
                                                 const Eigen::VectorXd&              g,
                                                 const Eigen::MatrixXd&              Ftipmat,
                                                 const std::vector<Eigen::MatrixXd>& Mlist,
                                                 const std::vector<Eigen::MatrixXd>& Glist,
                                                 const Eigen::MatrixXd&              Slist,
                                                 const Eigen::MatrixXd&              thetamatd,
                                                 const Eigen::MatrixXd&              dthetamatd,
                                                 const Eigen::MatrixXd&              ddthetamatd,
                                                 const Eigen::VectorXd&              gtilde,
                                                 const std::vector<Eigen::MatrixXd>& Mtildelist,
                                                 const std::vector<Eigen::MatrixXd>& Gtildelist,
                                                 double                              Kp,
                                                 double                              Ki,
                                                 double                              Kd,
                                                 double                              dt,
                                                 int                                 intRes)
    {
        Eigen::MatrixXd FtipmatT      = Ftipmat.transpose();
        Eigen::MatrixXd thetamatdT    = thetamatd.transpose();
        Eigen::MatrixXd dthetamatdT   = dthetamatd.transpose();
        Eigen::MatrixXd ddthetamatdT  = ddthetamatd.transpose();
        int             m             = thetamatdT.rows();
        int             n             = thetamatdT.cols();
        Eigen::VectorXd thetacurrent  = thetalist;
        Eigen::VectorXd dthetacurrent = dthetalist;
        Eigen::VectorXd eint          = Eigen::VectorXd::Zero(m);
        Eigen::MatrixXd taumatT       = Eigen::MatrixXd::Zero(m, n);
        Eigen::MatrixXd thetamatT     = Eigen::MatrixXd::Zero(m, n);
        Eigen::VectorXd taulist;
        Eigen::VectorXd ddthetalist;
        for (int i = 0; i < n; ++i)
        {
            taulist = ComputedTorque(thetacurrent, dthetacurrent, eint, gtilde, Mtildelist, Gtildelist, Slist, thetamatdT.col(i), dthetamatdT.col(i), ddthetamatdT.col(i), Kp, Ki, Kd);
            for (int j = 0; j < intRes; ++j)
            {
                ddthetalist = ForwardDynamics(thetacurrent, dthetacurrent, taulist, g, FtipmatT.col(i), Mlist, Glist, Slist);
                EulerStep(thetacurrent, dthetacurrent, ddthetalist, dt / intRes);
            }
            taumatT.col(i)   = taulist;
            thetamatT.col(i) = thetacurrent;
            eint += dt * (thetamatdT.col(i) - thetacurrent);
        }
        std::vector<Eigen::MatrixXd> ControlTauTraj_ret;
        ControlTauTraj_ret.push_back(taumatT.transpose());
        ControlTauTraj_ret.push_back(thetamatT.transpose());
        return ControlTauTraj_ret;
    }
} // namespace mr