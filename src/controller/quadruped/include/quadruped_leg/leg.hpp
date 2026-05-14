#ifndef LEG_HPP
#define LEG_HPP
#include "quadruped_common/enum_type.hpp"
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_math/math_ant.hpp"
#include "logger/logger.hpp"

class Leg
{
public:
    Leg() {}
    void Init(LegId leg_id, double l_1, double l_2, double l_3, Vec3D hip_trans_body);
    void CalcPos(Vec3D joint_q, FrameType frame);
    void CalcQ(Vec3D foot_pos, FrameType frame);
    double CalcQ1(const double &y, const double &z) const;
    double CalcQ2(const double &x, const double &z, const double &hip_foot_l) const;
    double CalcQ3(const double &hip_foot_l) const;

    Vec3D foot_pos_;
    Vec3D foot_q_;

    WaveState wave_state_;

    double l_1_;
    double l_2_;
    double l_3_;
    Vec3D init_bias_q_;
    Vec3I leg_side_;
protected:
    int leg_id_;
    Vec3D hip_trans_body_;
    Vec3D q_bias_;

};

class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, double abadLinkLength, double hipLinkLength, 
                  double kneeLinkLength, Vec3D pHip2B);
    ~QuadrupedLeg(){}
    Vec3D calcPEe2H(Vec3D q);
    Vec3D calcPEe2B(Vec3D q);
    Vec3D calcVEe(Vec3D q, Vec3D qd);
    Vec3D calcQ(Vec3D pEe, FrameType frame);
    Vec3D calcQd(Vec3D q, Vec3D vEe);
    Vec3D calcQd(Vec3D pEe, Vec3D vEe, FrameType frame);
    Vec3D calcTau(Vec3D q, Vec3D force);
    Mat3x3D calcJaco(Vec3D q);
    Vec3D getHip2B(){return _pHip2B;}
protected:
    double q1_ik(double py, double pz, double b2y);
    double q3_ik(double b3z, double b4z, double b);
    double q2_ik(double q1, double q3, double px, 
                 double py, double pz, double b3z, double b4z);
    double _sideSign;
    const double _abadLinkLength, _hipLinkLength, _kneeLinkLength;
    const Vec3D _pHip2B;
};


#endif