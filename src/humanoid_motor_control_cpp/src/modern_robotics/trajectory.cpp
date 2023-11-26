#include "modern_robotics/trajectory.h"

namespace mr
{
    double CubicTimeScaling(double Tf, double t)
    {
        double timeratio = 1.0 * t / Tf;
        double st        = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
        return st;
    }

    double QuinticTimeScaling(double Tf, double t)
    {
        double timeratio = 1.0 * t / Tf;
        double st        = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4) + 6 * pow(timeratio, 5);
        return st;
    }

    Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd& thetastart, const Eigen::VectorXd& thetaend, double Tf, int N, int method)
    {
        double          timegap = Tf / (N - 1);
        Eigen::MatrixXd trajT   = Eigen::MatrixXd::Zero(thetastart.size(), N);
        double          st;
        for (int i = 0; i < N; ++i)
        {
            if (method == 3)
                st = CubicTimeScaling(Tf, timegap * i);
            else
                st = QuinticTimeScaling(Tf, timegap * i);
            trajT.col(i) = st * thetaend + (1 - st) * thetastart;
        }
        Eigen::MatrixXd traj = trajT.transpose();
        return traj;
    }
    std::vector<Eigen::MatrixXd> ScrewTrajectory(const Eigen::MatrixXd& Xstart, const Eigen::MatrixXd& Xend, double Tf, int N, int method)
    {
        double                       timegap = Tf / (N - 1);
        std::vector<Eigen::MatrixXd> traj(N);
        double                       st;
        for (int i = 0; i < N; ++i)
        {
            if (method == 3)
                st = CubicTimeScaling(Tf, timegap * i);
            else
                st = QuinticTimeScaling(Tf, timegap * i);
            Eigen::MatrixXd Ttemp = MatrixLog6(TransInv(Xstart) * Xend);
            traj.at(i)            = Xstart * MatrixExp6(Ttemp * st);
        }
        return traj;
    }

    std::vector<Eigen::MatrixXd> CartesianTrajectory(const Eigen::MatrixXd& Xstart, const Eigen::MatrixXd& Xend, double Tf, int N, int method)
    {
        double                       timegap = Tf / (N - 1);
        std::vector<Eigen::MatrixXd> traj(N);
        std::vector<Eigen::MatrixXd> Rpstart = TransToRp(Xstart);
        std::vector<Eigen::MatrixXd> Rpend   = TransToRp(Xend);
        Eigen::Matrix3d              Rstart  = Rpstart[0];
        Eigen::Vector3d              pstart  = Rpstart[1];
        Eigen::Matrix3d              Rend    = Rpend[0];
        Eigen::Vector3d              pend    = Rpend[1];
        double                       st;
        for (int i = 0; i < N; ++i)
        {
            if (method == 3)
                st = CubicTimeScaling(Tf, timegap * i);
            else
                st = QuinticTimeScaling(Tf, timegap * i);
            Eigen::Matrix3d Ri = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend) * st);
            Eigen::Vector3d pi = st * pend + (1 - st) * pstart;
            Eigen::MatrixXd traji(4, 4);
            traji << Ri, pi, 0, 0, 0, 1;
            traj.at(i) = traji;
        }
        return traj;
    }
} // namespace mr
