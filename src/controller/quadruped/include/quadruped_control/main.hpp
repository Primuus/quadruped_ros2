#ifndef MAIN_HPP
#define MAIN_HPP

#include "quadruped_motor/motor_message.hpp"
#include "quadruped_control/hand.hpp"
#include "quadruped_fsm/fsm.hpp"
#include "logger/logger.hpp"


class Main {
public:
    Main();
    ~Main();
    
    bool initialize();
    void tick();
    void shutdown();
    void handleKeyboardInput(char key);
    void handleJoystickInput(double x1, double y1, double x2, double y2, const std::string& key);

    MotorSetting* getMotorSettings();

    
    Hand* hand_;
private:
    FSM* fsm_;
};

#endif // MAIN_HPP