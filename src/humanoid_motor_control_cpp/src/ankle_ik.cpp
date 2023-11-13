#include "humanoid_motor_control_cpp/ankle_ik.h"

#include <cassert>
#include <cmath>

namespace pi
{
    std::array<double, 2> ankle_ik(double d, double L1, double h1, double h2, double tx, double ty)
    {
        double cx = cos(tx);
        double sx = sin(tx);
        double cy = cos(ty);
        double sy = sin(ty);

        double AL = -cy * L1 * L1;
        double BL = L1 * h1 - cx * sy * L1 * L1 - sx * L1 * d;
        double CL = -(L1 * L1 + d * d - cx * d * d - sx * d * h1 - cx * sy * L1 * h1 + sx * sy * L1 * d);

        double LenL = sqrt(AL * AL + BL * BL);

        double AR = -cy * L1 * L1;
        double BR = L1 * h2 - cx * sy * L1 * L1 + sx * L1 * d;
        double CR = -(L1 * L1 + d * d - cx * d * d + sx * d * h2 - cx * sy * L1 * h2 - sx * sy * L1 * d);

        double LenR = sqrt(AR * AR + BR * BR);

        double tL_1 = asin(CL / LenL) - asin(AL / LenL);
        double tL_2 = asin(CL / LenL) + acos(BL / LenL);

        double tR_1 = asin(CR / LenR) - asin(AR / LenR);
        double tR_2 = asin(CR / LenR) + acos(BR / LenR);

        assert(fabs(tL_1 - tL_2) < 1e-3 && "tL_1 - tL_2 > 1e-3");
        assert(fabs(tR_1 - tR_2) < 1e-3 && "tR_1 - tR_2 > 1e-3");

        return {tL_1, tR_1};
    }
} // namespace pi
