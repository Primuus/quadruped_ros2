#include "quadruped_control/main.hpp"

Main::Main() : hand_(nullptr), fsm_(nullptr)
{
}

Main::~Main()
{
    shutdown();
}

bool Main::initialize()
{

    // 初始化硬件接口
    hand_ = new Hand();
    fsm_ = new FSM(hand_);

    hand_->Init();
    LOG_INFO("机器人开始运行");
    if (fsm_->state_list_ && fsm_->state_list_->current_state_)
    {
        fsm_->state_list_->current_state_->enter();
    }
    return true;
}

void Main::tick()
{
    fsm_->Run();
}

void Main::handleKeyboardInput(char key)
{
    if (!fsm_)
    {
        return;
    }
    if (key != '0')
    {
        hand_->kbd_cmd_ = key;
        // LOG_INFO("按键: %c", key);
    }
}

void Main::handleJoystickInput(double x1, double y1, double x2, double y2, const std::string& key)
{
    if (!fsm_ || !hand_)
    {
        return;
    }
    
    // 存储摇杆数据到 Hand 对象
    hand_->joystick_x1_ =  x1;
    hand_->joystick_y1_ = -y1;
    hand_->joystick_x2_ =  x2;
    hand_->joystick_y2_ = -y2;
    
    // 如果有按键，也存储按键
    if (!key.empty() && key[0] != '!')
    {
        hand_->kbd_cmd_ = key[0];
    }
    
    // LOG_INFO("摇杆数据: x1=%f, y1=%f, x2=%f, y2=%f", x1, y1, x2, y2);
    // LOG_INFO("按键: %c", hand_->kbd_cmd_);
    // TODO: 在这里添加具体的摇杆控制逻辑
    // 例如：根据摇杆数据调整机器人的运动方向、速度等
}

MotorSetting *Main::getMotorSettings()
{
    return hand_->motor_cmd_->motors_set_;
}

void Main::shutdown()
{
    LOG_INFO("系统退出");

    if (fsm_)
    {
        delete fsm_;
        fsm_ = nullptr;
    }

    if (hand_)
    {
        delete hand_;
        hand_ = nullptr;
    }
}

int main()
{
    Main controller;

    if (!controller.initialize())
    {
        LOG_ERROR("控制器初始化失败");
        return -1;
    }

    LOG_INFO("进入主控制循环");
    while (true)
    {
        controller.tick();
    }

    controller.shutdown();
    return 0;
}
