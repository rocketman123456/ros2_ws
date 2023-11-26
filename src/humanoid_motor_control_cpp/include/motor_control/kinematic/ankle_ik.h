#pragma once

#include <array>

namespace pi
{
    // d : 
    // L1 : 
    // h1 : 
    // h2 : 
    // tx : ankle target angle x, in rad
    // ty : ankle target angle y. in rad
    std::array<double, 2> ankle_ik(double d, double L1, double h1, double h2, double tx, double ty);
}
