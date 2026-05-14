#ifndef HAND_HPP
#define HAND_HPP
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_common/enum_type.hpp"
#include "quadruped_math/gait_param.hpp"
#include "quadruped_leg/leg.hpp"
#include "quadruped_motor/motor.hpp"
#include "quadruped_motion/motion.hpp"
#include "quadruped_phase/phase_manager.hpp"
#include "iostream"

class Hand
{
public:
    Hand() {}
    void Init();
    void UpdateInitQ();
    void SetZeroQ();
    void SetFeetPos(const Mat3x4D &pos);
    void SetFeetQ(const Mat3x4D &q);
    void CalcAngles();
    void UpdateTortParam(const Mat4x2F &tort, const Mat4x4D &delta);
    void UpdateCurrentState();
    void updateKbdCmd();
    
    void calcFeetPosByQ(const Mat3x4D &q);
    void calcFeetQByPos(const Mat3x4D &pos);
    Mat3x4D calcCurrentWithPhase(Mat3x4D enter,Mat3x4D target,Vec4D phase);


    // Debug
    void showFeetPos();
    void showFeetQ();
    void showPD();
    void showTau();
    void showReceivedState();
    void showPhase();
    void showInitQ();

    Mat3x4D feet_q_bias_;
    Mat3x4D feet_pos_;
    Mat3x4D feet_q_;
    Mat3x4D feet_dq_;
    Mat3x4D tauque_;

    GaitParams gait_params_;
    MotorCMD *motor_cmd_;
    PhaseManager *phase_manager_;
    bool first_enter_state_=true;
    bool inited_ = false;
    bool received_state_ = false;
    bool start_init_ = false;
    Leg *leg_[4];
    QuadrupedLeg *quadruped_leg_[4];
    bool update_param_ = false;
    // 键盘指令
    char kbd_cmd_;
    StateName next_state_n_;
    
    // 摇杆数据 (归一化到 [-1.0, 1.0])
    double joystick_x1_ = 0.0f;  // 左摇杆 X (左右)
    double joystick_y1_ = 0.0f;  // 左摇杆 Y (前后)
    double joystick_x2_ = 0.0f;  // 右摇杆 X
    double joystick_y2_ = 0.0f;  // 右摇杆 Y
    int x1_dir = 0;
    int y1_dir = 0;
    int x2_dir = 0;
    int y2_dir = 0;
    // 临时变量
    Vec3D foot_pos_;
    Vec3D foot_q_;

    // 機器人状态跟踪变量
    Mat3x4D this_epoch_position_;
    Mat3x4D this_epoch_velocity_;
    Mat3x4D current_position_;
    Mat3x4D current_velocity_;

    // IMU/状态数据
    Mat3x3D rot_mat_b2g_ = Mat3x3D::Identity();  // Body->Global旋转矩阵
    Vec3D gyro_global_ = Vec3D::Zero();          // 全局角速度
    Vec3D acc_global_ = Vec3D::Zero();           // 全局加速度
    double imu_quaternion_[4] = {1.0, 0.0, 0.0, 0.0};  // IMU四元数 [w,x,y,z]
    double imu_gyro_[3] = {0.0, 0.0, 0.0};       // IMU角速度
    double imu_acc_[3] = {0.0, 0.0, 0.0};       // IMU加速度
    double yaw_ = 0;                             // 当前偏航角
    double d_yaw_ = 0;                           // 偏航角速度
    
};
#endif
