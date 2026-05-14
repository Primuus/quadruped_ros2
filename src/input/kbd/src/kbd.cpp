#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using StringMsg = std_msgs::msg::String;

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        string_publisher_ = this->create_publisher<StringMsg>("/kbd_input", 10);
        
        // 设置非阻塞输入
        setup_keyboard();
    }
    
    ~KeyboardControl()
    {
        reset_terminal();
    }
    
    void run()
    {
        char key;
        RCLCPP_INFO(this->get_logger(), "按下键盘发出指令");
        while (rclcpp::ok())
        {
            key = read_key();
            
            // 发送字符命令到/quadruped_control端
            StringMsg::UniquePtr string_msg = std::make_unique<StringMsg>();
            
            if (key == 0) {
                // 无按键时不发送命令，只延时
                usleep(2000); // 2ms，500hz
                continue;
            }
            
            // 有按键时发送对应的按键字符
            string_msg->data = key;
            string_publisher_->publish(std::move(string_msg));
            
            RCLCPP_INFO(this->get_logger(), "按键: %c", key);
        }
    }

private:
    rclcpp::Publisher<StringMsg>::SharedPtr string_publisher_;
    struct termios orig_termios;
    
    void setup_keyboard()
    {
        struct termios new_termios;
        
        tcgetattr(STDIN_FILENO, &orig_termios);
        new_termios = orig_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }
    
    void reset_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }
    
    char read_key()
    {
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) != 1)
        {
            key = 0;
        }
        return key;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    
    try {
        node->run();
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "执行过程中发生错误");
    }
    
    rclcpp::shutdown();
    return 0;
}