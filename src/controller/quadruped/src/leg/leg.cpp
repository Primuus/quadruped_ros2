#include "quadruped_leg/leg.hpp"
/******************************************************************************
 * @brief 腿模型
 * @author antcolony
 * @date 2024-12-01
 ******************************************************************************/

/******************************
    outward->hip->knee->foot

    l_1 = outward_len
    l_2 = hip_len
    l_3 = knee_len

    q_1 = outward_angle
    q_2 = hip_angle
    q_3 = knee_angle
    acos -> 0 - PI
    asin -> -PI/2 - PI/2
    atan2 -> y / x
 ******************************/

using namespace std;
using namespace math;
// q3 大  角度大    q2 小 前  q1  小 右
void Leg::Init(LegId leg_id, double l_1, double l_2, double l_3, Vec3D hip_trans_body)
{
    leg_id_ = static_cast<int>(leg_id);
    if (leg_id == LegId::RightRear)
    {
        leg_side_ << -1, -1, +1;
        // init_bias_q_ << -19.5 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        init_bias_q_ << 0.0 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        // init_bias_q_.setZero();
    }
    else if (leg_id == LegId::RightFront)
    {
        leg_side_ << -1, -1, +1;
        // init_bias_q_ << -19.5 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        init_bias_q_ << 0.0 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        // init_bias_q_.setZero();
    }
    else if (leg_id == LegId::LeftRear)
    {
        leg_side_ << +1, -1, +1;
        // init_bias_q_ << -19.5 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        init_bias_q_ << 0.0 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        // init_bias_q_.setZero();
    }
    else if (leg_id == LegId::LeftFront)
    {
        leg_side_ << +1, -1, +1;
        // init_bias_q_ << -19.5f * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        init_bias_q_ << 0.0 * PI / 180.0, 71.2 * PI / 180.0, -161.5 * PI / 180.0;
        // init_bias_q_.setZero();
    }
    l_1_ = l_1;
    l_2_ = l_2;
    l_3_ = l_3;
    hip_trans_body_ = hip_trans_body;
    foot_pos_.setZero();
    foot_q_.setZero();
    q_bias_ << 0.0, -180.0, 0.0;
}
// 待完成
void Leg::CalcPos(Vec3D joint_q, FrameType frame)
{
    double sin_q_1 = sin(joint_q[0]);
    double cos_q_1 = cos(joint_q[0]);
    double sin_q_2 = sin(joint_q[1]);
    double cos_q_2 = cos(joint_q[1]);
    double sin_q_3 = sin(joint_q[2]);
    double cos_q_3 = cos(joint_q[2]);
    double cos_q_2plus3 = cos_q_2 * cos_q_3 - sin_q_2 * sin_q_3;
    double sin_q_2plus3 = sin_q_2 * cos_q_3 + cos_q_2 * sin_q_3;
    foot_pos_ << l_3_ * sin_q_2plus3 + l_2_ * sin_q_2,
                 -l_3_ * sin_q_1 * cos_q_2plus3 + l_1_ * cos_q_1 - l_2_ * cos_q_2 * sin_q_1,
                 l_3_ * cos_q_1 * cos_q_2plus3 + l_1_ * sin_q_1 + l_2_ * cos_q_1 * cos_q_2;
    if (frame == FrameType::BODY)
    {
        foot_pos_ = foot_pos_ + hip_trans_body_;
    }
}
void Leg::CalcQ(Vec3D foot_pos, FrameType frame_type)
{
    if (frame_type == FrameType::HIP)
    {
        foot_pos_ = foot_pos;
    }
    else
    {
        foot_pos_ = foot_pos - hip_trans_body_;
    }
    double x, y, z;
    x = foot_pos_(0);
    y = foot_pos_(1);
    z = foot_pos_(2);
    double whole_l = sqrt(SquareSum(x, y) + Square(z));
    double hip_foot_l = sqrt(SquareDiff(whole_l, l_1_));
    foot_q_(0) = leg_side_(0) * LimitQ1(CalcQ1(y, z)) +
                 leg_side_(0) * (q_bias_(0) * PI / 180.0 + init_bias_q_(0));
    foot_q_(1) = leg_side_(1) * LimitQ2(CalcQ2(x, z, hip_foot_l)) +
                 leg_side_(1) * (q_bias_(1) * PI / 180.0 + init_bias_q_(1));
    foot_q_(2) = leg_side_(2) * LimitQ3(CalcQ3(hip_foot_l)) +
                 leg_side_(2) * (q_bias_(2) * PI / 180.0 + init_bias_q_(2));
}
double Leg::CalcQ1(const double &y, const double &z) const
{
     double q_1;
     double L = sqrt(SquareSum(y, z) - Square(l_1_));
    if (y > l_1_)
    {
        q_1 = atan2(z * l_1_ + y * L, y * l_1_ - z * L);
        return q_1;
    }
    else if (y < l_1_)
    {
        q_1 = atan2(-z * l_1_ - y * L, y * l_1_ - z * L);
        return -q_1;
    }
    else
    {
        return 0.0;
    }
}
double Leg::CalcQ2(const double &x, const double &z, const double &hip_foot_l) const
{
    Vec2D q_2;
    Vec2D q_a, q_b;
    if (x == 0)
    {
        q_2(1) = (SquareSum(hip_foot_l, l_2_) - Square(l_3_)) / (2 * hip_foot_l * l_2_);
        Limit(&q_2(1), 1.0);
        q_2(0) = acos(q_2(1));
        return PI / 2.0 - q_2(0);
    }
    if (x > 0)
    {
        if (z < 0)
        {
            // z = -z;
            q_a(0) = atan2(-z, x);
        }
        else
        {
            q_a(0) = atan2(z, x);
        }
        q_b(1) = (SquareSum(l_2_, hip_foot_l) - Square(l_3_)) / (2 * l_2_ * hip_foot_l);
        Limit(&q_b(1), 1.0);
        q_b(0) = acos(q_b(1));
        q_2(0) = PI - (q_a(0) + q_b(0));
    }
    else if (x < 0)
    {
        if (z < 0)
        {
            // z = -z;x = -x
            q_a(0) = atan2(-z, -x);
        }
        else
        {
            q_a(0) = atan2(z, -x);
        }
        q_b(1) = (SquareSum(l_2_, hip_foot_l) - Square(l_3_)) / (2 * l_2_ * hip_foot_l);
        Limit(&q_b(1), 1.0);
        q_b(0) = acos(q_b(1));
        q_2(0) = q_a(0) - q_b(0);
    }
    return q_2(0);
}
double Leg::CalcQ3(const double &hip_foot_l) const
{
    double q_3, temp;
    temp = (SquareSum(l_2_, l_3_) - Square(hip_foot_l)) / (2 * l_2_ * l_3_);
    Limit(&temp, 1.0);
    q_3 = acos(temp) - 40.0 * PI / 180.0;
    return q_3;
}



/************************/
/*******QuadrupedLeg*****/
/************************/
QuadrupedLeg::QuadrupedLeg(int legID, double abadLinkLength, double hipLinkLength, 
                            double kneeLinkLength, Vec3D pHip2B)
            :_abadLinkLength(abadLinkLength), 
             _hipLinkLength(hipLinkLength), 
             _kneeLinkLength(kneeLinkLength), 
             _pHip2B(pHip2B){
    if (legID == 0 || legID == 2)
        _sideSign = -1;
    else if (legID == 1 || legID == 3)
        _sideSign = 1;
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }
}

// Forward Kinematics
Vec3D QuadrupedLeg::calcPEe2H(Vec3D q){
     double l1 = _sideSign * _abadLinkLength;
     double l2 = -_hipLinkLength;
     double l3 = -_kneeLinkLength;

     double s1 = std::sin(q(0));
     double s2 = std::sin(q(1));
     double s3 = std::sin(q(2));

     double c1 = std::cos(q(0));
     double c2 = std::cos(q(1));
     double c3 = std::cos(q(2));

     double c23 = c2 * c3 - s2 * s3;
     double s23 = s2 * c3 + c2 * s3;

    Vec3D pEe2H;

    pEe2H(0) = l3 * s23 + l2 * s2;
    pEe2H(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    pEe2H(2) =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return pEe2H;
}

// Forward Kinematics
Vec3D QuadrupedLeg::calcPEe2B(Vec3D q){
    return _pHip2B + calcPEe2H(q);
}

// Derivative Forward Kinematics
Vec3D QuadrupedLeg::calcVEe(Vec3D q, Vec3D qd){
    return calcJaco(q) * qd;
}

// Inverse Kinematics
Vec3D QuadrupedLeg::calcQ(Vec3D pEe, FrameType frame){
    Vec3D pEe2H;
    if(frame == FrameType::HIP)
        pEe2H = pEe;
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    double q1, q2, q3;
    Vec3D qResult;
    double px, py, pz;
    double b2y, b3z, b4z, a, b, c;

    px = pEe2H(0);
    py = pEe2H(1);
    pz = pEe2H(2);

    b2y = _abadLinkLength * _sideSign;
    b3z = -_hipLinkLength;
    b4z = -_kneeLinkLength;
    a = _abadLinkLength;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    q1 = q1_ik(py, pz, b2y);
    q3 = q3_ik(b3z, b4z, b);
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

    qResult(0) = q1;
    qResult(1) = q2;
    qResult(2) = q3;

    return qResult;
}

// Derivative Inverse Kinematics
Vec3D QuadrupedLeg::calcQd(Vec3D q, Vec3D vEe){
    return calcJaco(q).inverse() * vEe;
}

// Derivative Inverse Kinematics
Vec3D QuadrupedLeg::calcQd(Vec3D pEe, Vec3D vEe, FrameType frame){
    Vec3D q = calcQ(pEe, frame);
    return calcJaco(q).inverse() * vEe;
}

// Inverse Dynamics
Vec3D QuadrupedLeg::calcTau(Vec3D q, Vec3D force){
    return calcJaco(q).transpose() * force;
}

// Jacobian Matrix
Mat3x3D QuadrupedLeg::calcJaco(Vec3D q){
    Mat3x3D jaco;
    jaco.setZero();
    double l1 = _abadLinkLength * _sideSign;
    double l2 = -_hipLinkLength;
    double l3 = -_kneeLinkLength;

    double s1 = std::sin(q(0));
    double s2 = std::sin(q(1));
    double s3 = std::sin(q(2));

    double c1 = std::cos(q(0));
    double c2 = std::cos(q(1));
    double c3 = std::cos(q(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;
    jaco(0, 0) = 0;
    jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jaco(0, 1) = l3 * c23 + l2 * c2;
    jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    jaco(0, 2) = l3 * c23;
    jaco(1, 2) = l3 * s1 * s23;
    jaco(2, 2) = -l3 * c1 * s23;

    return jaco;
}

double QuadrupedLeg::q1_ik(double py, double pz, double l1){
    double q1;
    double L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

double QuadrupedLeg::q3_ik(double b3z, double b4z, double b){
    double q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    q3 = -(M_PI - q3); //0~180
    return q3;
}

double QuadrupedLeg::q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z){
    double q2, a1, a2, m1, m2;
    
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}
