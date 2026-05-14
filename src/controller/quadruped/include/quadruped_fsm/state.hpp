#ifndef STATE_HPP
#define STATE_HPP
#include "quadruped_common/eigen_type.hpp"
#include "quadruped_common/enum_type.hpp"
#include "quadruped_control/hand.hpp"
#include "quadruped_motion/motion.hpp"
#include "logger/logger.hpp"
#include "quadruped_config/tort_config.hpp"
#include "quadruped_config/jump_config.hpp"
#include "quadruped_config/free_angle_config.hpp"
#include "quadruped_config/fixed_stand_config.hpp"
#include "quadruped_config/init_config.hpp"
#include "quadruped_config/turn_config.hpp"

namespace base
{
    class State
    {
    public:
        State(Hand *hand) : hand_(hand)
        {
        }
        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        void Init()
        {
            mc_ = new MotionController;
            enter_pos_ = hand_->feet_pos_;
            // ImuEnterState(dir_);
            z.setZero();
            for (int i = 0; i < 4; i++)
            {
                base_pos_.col(i) << -0.02, 0.18, -0.24;
                phase_t_(i) = 0.7;
                target_tort_param_.row(i) << 0.5, 1.0;
                target_delta_param_.row(i) << 0.0, 0.0, 0.0, 6.0;
                kp_.col(i) << 10.0, 10.0, 10.0;
                kd_.col(i) << 0.25, 0.25, 0.25;
            }
            hand_->UpdateTortParam(target_tort_param_, target_delta_param_);
            hand_->gait_params_.bias_ << 0.0, 0.5, 0.5, 0.0;
        }
        Hand *hand_;
        MotionController *mc_;
        Mat3x4D z;
        Mat3x4D kp_;
        Mat3x4D kd_;
        Mat3x4D enter_pos_;
        Mat3x4D base_pos_;
        Vec4D phase_t_;
        double dir_[3];

        Mat4x2F target_tort_param_; // 设定值
        Mat4x4D target_delta_param_;

        bool valuable_change_ = true;
            
    };
}

#define BASE_STATE_FUNCTION                   \
    public:                                   \
        State(Hand *hand) : base::State(hand) \
        {                                     \
        }                                     \
        ~State()                              \
        {                                     \
        }                                     \
        void enter();                         \
        void run();                           \
        void exit();


namespace init
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION
        bool stand_ = false;
        bool stand_prepare_ = false;
        Mat3x4D enter_q_;
        Mat3x4D target_q_;
        
        init::ConfigLoader config_loader_;
    };
} // namespace init
namespace deinit
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION
    };
} // namespace deinit
namespace tort
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION
        uint8_t stage_ = 0;
        tort::ConfigLoader config_loader_{"stand"};
        std::string current_config_type_ = "stand";

        Mat3x4D enter_q_;
        Mat3x4D target_foot_pos_;
        Mat3x4D enter_foot_pos_;
        
    };
} // namespace tort

namespace jump
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION

        jump::ConfigLoader config_loader_;
        std::vector<jump::JumpStage> stages_;
        int current_stage_ = 0;
        
    private:
        void applyCurrentStage();
    };
} // namespace jump

namespace free_angle
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION

        Mat3x4D enter_q_;
        Mat3x4D target_q_;
        
        free_angle::ConfigLoader config_loader_;
    };
} // namespace free_angle

namespace fixed_stand
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION

        Mat3x4D enter_q_;
        Mat3x4D target_foot_pos_;
        Mat3x4D enter_foot_pos_;
        
        fixed_stand::ConfigLoader config_loader_;
    };
} // namespace fixed_stand


namespace turn
{
    class State : public base::State
    {
        BASE_STATE_FUNCTION
        turn::ConfigLoader config_loader_;
        uint8_t stage_ = 0;
        Mat3x4D enter_q_;
        Mat3x4D target_foot_pos_;
        Mat3x4D enter_foot_pos_;
        
    };
} // namespace turn


#endif
