#ifndef GAIT_PARAM_HPP
#define GAIT_PARAM_HPP
#include "quadruped_common/eigen_type.hpp"

class GaitParams
{
public:
    Mat4x2F tort_param_; // swing_radio,peroid
    Mat4x4D delta_;      // x,y,z,h
    Vec4D bias_;
};

#endif
