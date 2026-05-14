#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_math/math_ant.hpp"
using namespace std;
inline Vec3D EqualPos(const Vec3D &pos)
{
    return pos;
}
inline Vec3D EqualPos(int leg_id, const Mat3x4D &pos)
{
    Vec3D result;
    result(0) = pos(0, leg_id);
    result(1) = pos(1, leg_id);
    result(2) = pos(2, leg_id);
    return result;
}

inline Vec3D AddDeltaPos(const Vec3D &pos, const Vec3D &delta)
{
    return pos + delta;
}

inline Vec3D AddDeltaPos(int leg_id, const Mat3x4D &pos, const Mat3x4D &delta)
{
    Vec3D result;
    result(0) = pos(0, leg_id) + delta(0, leg_id);
    result(1) = pos(1, leg_id) + delta(1, leg_id);
    result(2) = pos(2, leg_id) + delta(2, leg_id);
    return result;
}

inline double CalcLinear1DPos(const double &start, const double &end, const double &phase)
{
    return start + (end - start) * phase;
}

inline Vec2D CalcLinear2DPos(const Vec2D &start, const Vec2D &end, const double &phase)
{
    return start + (end - start) * phase;
}

inline Vec3D CalcLinear3DPos(const Vec3D &start, const Vec3D &end, const double &phase)
{
    return start + (end - start) * phase;
}

// 单轴x + sinPIx轨迹
inline double CalcSinPos(const double &start, const double &end, const double h, const double &phase)
{
    return start + (end - start) * phase + h * sin(PI * phase);
}
// 单轴0.5sin轨迹
inline double CalcHalfSinPos(const double &start, const double &end, const double h, const double &phase)
{
    return start + (end - start + h) * sin(PI * phase / 2.0);
}
// 单轴(1-cospi)/2轨迹
inline double CalcCosPos(const double &start, const double &end, const double &phase)
{
    return start + (end - start) * phase * ((1.0 - cos(PI * phase)) / 2.0);
}

inline double CalcCycloidXPos(const double &start, const double &end, const double &phase)
{
    double result;
    result = start + (end - start) * 1.0 / (2.0 * PI) * (2 * PI * phase - sin(2 * PI * phase));
    return result;
}

inline double CalcCycloidYPos(const double &start, const double &end, const double &phase)
{
    double result;
    result = start + (end - start) * 1.0 / (2.0 * PI) * (2 * PI * phase - sin(2 * PI * phase));
    return result;
}

inline double CalcCycloidZPos(const double &start, const double &h, const double &phase)
{
    double result;
    if (phase < 0.5 && phase >= 0)
    {
        result = start + h * (2 * (4 * pow(phase, 3) - 6 * pow(phase, 2) + 3 * phase));
    }
    else if (phase >= 0.5 && phase <= 1)
    {
        result = start + h * (2 * (-4 * pow(phase, 3) + 6 * pow(phase, 2) - 3 * phase + 1));
    }
    return result;
}

inline double ClacLandZPos(const double &start, const double &h, const double &phase)
{
    double result;
    if (phase < 0.5 && phase >= 0)
    {
        result = start - (h * (1.0 / (2 * PI)) * (2 * PI * phase * 2.0 - sin(2 * PI * phase * 2.0)));
    }
    else if (phase >= 0.5 && phase <= 1)
    {
        result = start - (h * (1.0 / (2 * PI)) * (2 * PI * (2.0 - phase * 2.0) - sin(2 * PI * (2.0 - phase * 2.0))));
    }
    return result;
}

inline double ClacPreLandZPos(const double &start, const double &h, const double &phase)
{
    double result;
    result = start - (h * (1.0 / (2 * PI)) * (PI * phase * 2.0 - sin(2 * PI * phase)));
    return result;
}

inline double ClacEndLandZPos(const double &start, const double &h, const double &phase)
{
    double result;
    result = start - (h * (1.0 / (2 * PI)) * (2 * PI * (1.0 - phase) - sin(2 * PI * (1.0 - phase))));
    return result;
}

inline double CalcStepZpos(const double &start, const double &end, const double &h, const double &phase)
{
    double result;
    result = start + (end - start) * phase + h * sqrt(1 - pow(2 * phase - 1, 2));
    return result;
}
// 基础运动函数集 - 单腿运动
inline Vec3D MoveLegLinear(const Vec3D &start_pos, const Vec3D &end_pos, double phase)
{
    return CalcLinear3DPos(start_pos, end_pos, phase);
}

inline Vec3D MoveLegCycloidXY(const Vec3D &start_pos, const Vec3D &end_pos, double phase)
{
    Vec3D result;
    result(0) = CalcCycloidXPos(start_pos(0), end_pos(0), phase);
    result(1) = CalcCycloidYPos(start_pos(1), end_pos(1), phase);
    result(2) = start_pos(2);
    return result;
}

inline Vec3D MoveLegWithSwingHeight(const Vec3D &start_pos, const Vec3D &end_pos, double height, double phase)
{
    Vec3D result;
    result(0) = CalcCycloidXPos(start_pos(0), end_pos(0), phase);
    result(1) = CalcCycloidYPos(start_pos(1), end_pos(1), phase);
    result(2) = CalcCycloidZPos(start_pos(2), height, phase);
    return result;
}

inline Vec3D MoveLegWithStepHeight(const Vec3D &start_pos, const Vec3D &end_pos, double height, double phase)
{
    Vec3D result;
    result(0) = CalcCycloidXPos(start_pos(0), end_pos(0), phase);
    result(1) = CalcCycloidYPos(start_pos(1), end_pos(1), phase);
    result(2) = CalcStepZpos(start_pos(2), end_pos(2), height, phase);
    return result;
}

// 增量计算方法 - 单腿运动
inline Vec3D MoveLegLinearIncrement(const Vec3D &start_pos, const Vec3D &delta, double height, double phase)
{
    // Vec3D result;
    // result = start_pos + (delta * 1.0f / (2.0f * PI)) * (2 * PI * phase - sin(2 * PI * phase));
    // return result;
    return CalcLinear3DPos(start_pos, start_pos + delta, phase);
}

// inline Vec3D MoveLegCycloidXYIncrement(const Vec3D &start_pos, const Vec3D &delta, float height, float phase)
// {
//     Vec3D result;
//     result(0) = CalcCycloidXPos(start_pos(0), start_pos(0) + delta(0), phase);
//     result(1) = CalcCycloidYPos(start_pos(1), start_pos(1) + delta(1), phase);
//     result(2) = start_pos(2);
//     return result;
// }

inline Vec3D MoveLegWithSwingHeightIncrement(const Vec3D &start_pos, const Vec3D &delta, double height, double phase)
{
    Vec3D result;
    if (delta(2) > 0.5 || delta(2) < -0.5)
    {
        result(0) = CalcCycloidXPos(start_pos(0), start_pos(0) + delta(0), phase);
        result(1) = CalcCycloidYPos(start_pos(1), start_pos(1) + delta(1), phase);
        result(2) = CalcStepZpos(start_pos(2), start_pos(2) + delta(2), height, phase);
    }
    else
    {
        result(0) = CalcCycloidXPos(start_pos(0), start_pos(0) + delta(0), phase);
        result(1) = CalcCycloidYPos(start_pos(1), start_pos(1) + delta(1), phase);
        result(2) = CalcCycloidZPos(start_pos(2), height, phase);
    }
    return result;
}

inline Vec3D MoveLegWithLandHeightIncrement(const Vec3D &start_pos, const Vec3D &delta, double height, double phase)
{
    Vec3D result;
    result(0) = CalcCycloidXPos(start_pos(0), start_pos(0) + delta(0), phase);
    result(1) = CalcCycloidYPos(start_pos(1), start_pos(1) + delta(1), phase);
    result(2) = ClacLandZPos(start_pos(2), height, phase);
    return result;
}

inline Vec3D MoveLegWithStepHeightIncrement(const Vec3D &start_pos, const Vec3D &delta, double height, double phase)
{
    Vec3D result;
    result(0) = CalcCycloidXPos(start_pos(0), start_pos(0) + delta(0), phase);
    result(1) = CalcCycloidYPos(start_pos(1), start_pos(1) + delta(1), phase);
    result(2) = CalcStepZpos(start_pos(2), start_pos(2) + delta(2), height, phase);
    return result;
}

inline Vec3D MoveLegAngleIncrement(const Vec3D &start_angle, const Vec3D &delta_angle, double height, double phase)
{
    Vec3D result;
    result(0) = start_angle(0) + delta_angle(0) * phase;
    result(1) = start_angle(1) + delta_angle(1) * phase;
    result(2) = start_angle(2) + delta_angle(2) * phase;
    return result;
}

#endif // TRAJECTORY_HPP