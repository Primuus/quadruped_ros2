#include "quadruped_control/hand.hpp"
#include "quadruped_math/trajectory.hpp"
#include "quadruped_motion/motion.hpp"
#include "logger/logger.hpp"

void Hand::Init()
{
    motor_cmd_ = new MotorCMD;
    phase_manager_ = new PhaseManager;
    feet_pos_.setZero();
    foot_pos_.setZero();
    gait_params_.bias_.setZero();
    feet_q_bias_.col(0) << 0.0, 0.0, 0.0;
    feet_q_bias_.col(1) << 0.0, 0.0, 0.0;
    feet_q_bias_.col(2) << 0.0, 0.0, 0.0;
    feet_q_bias_.col(3) << 0.0, 0.0, 0.0;
    
    // 初始化新增的状态跟踪变量
    current_position_.setZero();
    current_velocity_.setZero();
    this_epoch_position_.setZero();
    this_epoch_velocity_.setZero();
    feet_q_.setZero();
    feet_dq_.setZero();

    tauque_.setZero();
    
    // unitree约定: leg0=FR, leg1=FL, leg2=RR, leg3=RL
    // LegId: RightFront=0, LeftFront=1, RightRear=2, LeftRear=3
    for (int i = 0; i < 4; i++)
    {
        leg_[i] = new Leg;
        if (i == 0) // RightFront (FR)
            leg_[i]->Init(LegId(i), 12.5f, 22.5f, 29.5f, Vec3D(0.1881, -0.04675, 0));
        else if (i == 1) // LeftFront (FL)
            leg_[i]->Init(LegId(i), 12.5f, 22.5f, 29.5f, Vec3D(0.1881,  0.04675, 0));
        else if (i == 2) // RightRear (RR)
            leg_[i]->Init(LegId(i), 12.5f, 22.5f, 29.5f, Vec3D(-0.1881, -0.04675, 0));
        else if (i == 3) // LeftRear (RL)
            leg_[i]->Init(LegId(i), 12.5f, 22.5f, 29.5f, Vec3D(-0.1881,  0.04675, 0));
    }
    // legID决定sideSign: 偶数legID=0,2 → sideSign=-1(右腿), 奇数legID=1,3 → sideSign=+1(左腿)
    quadruped_leg_[0] = new QuadrupedLeg(0, 0.08, 0.213, 0.213, Vec3D( 0.1881, -0.04675, 0));  // FR
    quadruped_leg_[1] = new QuadrupedLeg(1, 0.08, 0.213, 0.213, Vec3D( 0.1881,  0.04675, 0));  // FL
    quadruped_leg_[2] = new QuadrupedLeg(2, 0.08, 0.213, 0.213, Vec3D(-0.1881, -0.04675, 0));  // RR
    quadruped_leg_[3] = new QuadrupedLeg(3, 0.08, 0.213, 0.213, Vec3D(-0.1881,  0.04675, 0));  // RL
    kbd_cmd_ =  '!';
    next_state_n_ = StateName::Invaild;
    update_param_ = false;
}

void Hand::UpdateInitQ()
{
    for (int i(0); i < 4; ++i)
    {
        leg_[i]->init_bias_q_(0) = this_epoch_position_(0,i);
        leg_[i]->init_bias_q_(1) = this_epoch_position_(1,i);
        leg_[i]->init_bias_q_(2) = this_epoch_position_(2,i);
    }
}

void Hand::SetZeroQ()
{
    for (int i(0); i < 4; ++i)
    {
        feet_q_(0,i) = leg_[i]->init_bias_q_(0);
        feet_q_(1,i) = leg_[i]->init_bias_q_(1);
        feet_q_(2,i) = leg_[i]->init_bias_q_(2);
    }
}

void Hand::SetFeetPos(const Mat3x4D &pos)
{
    for (int i(0); i < 4; ++i)
    {
        feet_pos_(0,i) = pos(0,i);
        feet_pos_(1,i) = pos(1,i);
        feet_pos_(2,i) = pos(2,i);
    }
}

void Hand::SetFeetQ(const Mat3x4D &q)
{
    for (int i(0); i < 4; ++i)
    {
        feet_q_(0,i) = q(0,i);
        feet_q_(1,i) = q(1,i);
        feet_q_(2,i) = q(2,i);
    }
}


void Hand::UpdateCurrentState()
{
    if (received_state_)
    {
        this_epoch_position_ = current_position_;
        this_epoch_velocity_ = current_velocity_;
    }
}


void Hand::UpdateTortParam(const Mat4x2F &tort, const Mat4x4D &delta)
{
    for (int i(0); i < LEG_NUM; ++i)
    {
        gait_params_.tort_param_.row(i) = tort.row(i);
        gait_params_.delta_.row(i) = delta.row(i);
    }
}

void Hand::updateKbdCmd()
{
    next_state_n_ = StateName::Invaild;
    if (kbd_cmd_ != '!')
    {
        switch (kbd_cmd_)
        {
            case 'b': 
            next_state_n_ = StateName::Tort;
            break;
            case 'h':
            next_state_n_ = StateName::FixedStand;
            break;
            case 'g':
            next_state_n_ = StateName::Jump;
            break;
            case 'i':
            next_state_n_ = StateName::Deinit;
            break;
            case 'F':
            next_state_n_ = StateName::Init;
            break;
            case 'a':
            next_state_n_ = StateName::FreeAngle;
            break;
            case 'd':
            LOG_INFO("更新参数");
            update_param_ = true;
            break;
            case 'f':
            start_init_ = true;
            LOG_INFO("收到f，允许Init起身");
            break;
            case 'u':
            next_state_n_ = StateName::FreeAngle;
            break;
            case 'c':
            next_state_n_ = StateName::Jump;
            break;
        }
        kbd_cmd_ = '!';
    }

    if (joystick_x1_ > 0.5)      x1_dir = 1;
    else if (joystick_x1_ < -0.5) x1_dir = -1;
    else x1_dir = 0;

    if (joystick_y1_ > 0.5)      y1_dir = 1;
    else if (joystick_y1_ < -0.5) y1_dir = -1;
    else y1_dir = 0;

    if (joystick_x2_ > 0.5)      x2_dir = 1;
    else if (joystick_x2_ < -0.5) x2_dir = -1;
    else x2_dir = 0;

    if (joystick_y2_ > 0.5)      y2_dir = 1;
    else if (joystick_y2_ < -0.5) y2_dir = -1;
    else y2_dir = 0;

}

void Hand::CalcAngles()
{
    for (int i(0); i < LEG_NUM; i++)
    {
        foot_pos_ = feet_pos_.col(i);
        // 老结算
        // leg_[i]->CalcQ(foot_pos_, FrameType::HIP);
        // feet_q_.col(i) = leg_[i]->foot_q_;
        // feet_q_.col(i) = feet_q_.col(i) + feet_q_bias_.col(i) * PI / 180.0f;
        // 新结算
        foot_q_ = quadruped_leg_[i]->calcQ(foot_pos_, FrameType::HIP);
        feet_q_.col(i) = foot_q_;
        foot_pos_ = quadruped_leg_[i]->calcPEe2H(foot_q_);
        feet_pos_.col(i) = foot_pos_;
        feet_q_.col(i) = feet_q_.col(i) + feet_q_bias_.col(i) * PI / 180.0f;
    }
    
    // showFeetQ();
    // showInitQ();
    // showReceivedState();
    // showTau();
    motor_cmd_->CalcTau(feet_q_, this_epoch_position_, feet_dq_, this_epoch_velocity_, tauque_);
}

Mat3x4D Hand::calcCurrentWithPhase(Mat3x4D enter,Mat3x4D target,Vec4D phase)
{
    Mat3x4D result;
    for (int i(0); i < 4; i++)
    {
        result.col(i) = (target.col(i) - enter.col(i)) * phase(i) + enter.col(i);
    }
    return result;
}

void Hand::calcFeetPosByQ(const Mat3x4D &q)
{
    for (int i = 0; i < 4; i++)
    {
        foot_q_ = q.col(i);
        foot_pos_ = quadruped_leg_[i]->calcPEe2H(foot_q_);
        feet_pos_.col(i) = foot_pos_;
    }
}

void Hand::calcFeetQByPos(const Mat3x4D &pos)
{
    for (int i = 0; i < 4; i++)
    {
        foot_pos_ = pos.col(i);
        foot_q_ = quadruped_leg_[i]->calcQ(foot_pos_,FrameType::HIP);
        feet_q_.col(i) = foot_q_;
    }
}





// Debug
void Hand::showFeetPos()
{
    LOG_DEBUG("足端位置");
    for (int i(0); i < 4; i++)
    {
        LOG_DEBUG("[%d] %f %f %f", i, feet_pos_(0,i), feet_pos_(1,i), feet_pos_(2,i));
    }
}

void Hand::showFeetQ()
{
    LOG_INFO("执行指令");
    for (int i = 0; i < LEG_NUM; i++)
    {
        LOG_INFO("[%d] %f,%f,%f",i,(feet_q_(0,i) - leg_[i]->init_bias_q_(0)*leg_[i]->leg_side_(0)) * 180 / 3.14159,(feet_q_(1,i) - leg_[i]->init_bias_q_(1)*leg_[i]->leg_side_(1)) * 180 / 3.14159,(feet_q_(2,i) - leg_[i]->init_bias_q_(2)*leg_[i]->leg_side_(2)) * 180 / 3.14159); 
    }
}

void Hand::showPD()
{
    LOG_INFO("PD指令");
    for (int i = 0; i < LEG_NUM; i++)
    {
        LOG_INFO("[%d] %f,%f,%f,%f,%f,%f",i,motor_cmd_->motors_set_[i*3+0].Kp_,motor_cmd_->motors_set_[i*3+0].Kd_,motor_cmd_->motors_set_[i*3+1].Kp_,motor_cmd_->motors_set_[i*3+1].Kd_,motor_cmd_->motors_set_[i*3+2].Kp_,motor_cmd_->motors_set_[i*3+2].Kd_); 
    }
}

void Hand::showTau()
{
    LOG_INFO("力矩指令");
    LOG_INFO("[0] %f,%f,%f",motor_cmd_->motors_set_[0].tau_,motor_cmd_->motors_set_[1].tau_,motor_cmd_->motors_set_[2].tau_); 
    LOG_INFO("[1] %f,%f,%f",motor_cmd_->motors_set_[3].tau_,motor_cmd_->motors_set_[4].tau_,motor_cmd_->motors_set_[5].tau_); 
    LOG_INFO("[2] %f,%f,%f",motor_cmd_->motors_set_[6].tau_,motor_cmd_->motors_set_[7].tau_,motor_cmd_->motors_set_[8].tau_); 
    LOG_INFO("[3] %f,%f,%f",motor_cmd_->motors_set_[9].tau_,motor_cmd_->motors_set_[10].tau_,motor_cmd_->motors_set_[11].tau_); 
}

void Hand::showReceivedState()
{
    LOG_INFO("接收到的角度状态");
    // for (int i = 0; i < LEG_NUM; i++)
    // {
    //     LOG_INFO("%f,%f,%f",(this_epoch_position_(0,i) - leg_[i]->init_bias_q_(0)*leg_[i]->leg_side_(0)) * 180 / 3.14159,(this_epoch_position_(1,i) - leg_[i]->init_bias_q_(1)*leg_[i]->leg_side_(1)) * 180 / 3.14159,(this_epoch_position_(2,i) - leg_[i]->init_bias_q_(2)*leg_[i]->leg_side_(2)) * 180 / 3.14159); 
    // }
    for (int i = 0; i < LEG_NUM; i++)
    {
        LOG_INFO("%f,%f,%f",this_epoch_position_(0,i),this_epoch_position_(1,i),this_epoch_position_(2,i)); 
    }
}

void Hand::showPhase()
{
    LOG_INFO("相位周期信息");
    for (int i = 0; i < LEG_NUM; i++)
    {
        LOG_INFO("[%d] 当前周期%f,经过时间%f,相位%f",i,phase_manager_->phase_t_(i),phase_manager_->passed_t_(i),phase_manager_->phase_(i)); 
    }
}
void Hand::showInitQ()
{
    LOG_INFO("初始化角度信息");
    for (int i = 0; i < LEG_NUM; i++)
    {
        LOG_INFO("[%d] %f,%f,%f",i,leg_[i]->init_bias_q_(0)*180.0/PI,leg_[i]->init_bias_q_(1)*180.0/PI,leg_[i]->init_bias_q_(2)*180.0/PI); 
    }
}
