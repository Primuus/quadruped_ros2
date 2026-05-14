#include "quadruped_config/tort_config.hpp"
#include <fstream>
#include <sstream>

namespace tort {

ConfigLoader::ConfigLoader(const std::string& config_type)
{
    setConfigType(config_type);
    base_pos_config_.setZero();
    kp_config_.setZero();
    kd_config_.setZero();
    tort_param_config_.setZero();
    delta_param_config_.setZero();
    phase_config_.setZero();
}

void ConfigLoader::setConfigType(const std::string& config_type)
{
    if (config_type == "back")
    {
        config_file_ = "src/controller/quadruped/config/tort_back_params.yaml";
    }
    else if (config_type == "walk")
    {
        config_file_ = "src/controller/quadruped/config/tort_params.yaml";
    }
    else if (config_type == "left")
    {
        config_file_ = "src/controller/quadruped/config/turn_left_params.yaml";
    }
    else if (config_type == "right")
    {
        config_file_ = "src/controller/quadruped/config/turn_right_params.yaml";
    }
    else
    {
        config_file_ = "src/controller/quadruped/config/tort_stand_params.yaml";
    }
}

bool ConfigLoader::loadConfig() {
    std::ifstream file(config_file_);
    if (!file.is_open()) {
        std::cout << "无法打开配置文件: " << config_file_ << std::endl;
        return false;
    }
    
    // std::cout << "成功打开配置文件: " << config_file_ << std::endl;
    
    std::string line;
    std::string content;
    
    while (std::getline(file, line)) {
        content += line + "\n";
    }
    file.close();
    
    // 解析各个部分
    bool success = true;
    success &= parseBasePosition(content);
    success &= parseKpParams(content);
    success &= parseKdParams(content);
    success &= parseTortParams(content);
    success &= parseDeltaParams(content);
    success &= parsePhaseValues(content);

    // std::cout << "BasePosition: \n" << base_pos_config_ << std::endl;
    // std::cout << "Kp: \n" << kp_config_ << std::endl;
    // std::cout << "Kd: \n" << kd_config_ << std::endl;
    // std::cout << "TortParam: \n" << tort_param_config_ << std::endl;
    // std::cout << "DeltaParam: \n" << delta_param_config_ << std::endl;
    // std::cout << "Phase: \n" << phase_config_ << std::endl;

    
    if (success) {
        // std::cout << "配置文件解析成功: " << config_file_ << std::endl;
        return true;
    } else {
        std::cout << "配置文件解析失败: " << config_file_ << std::endl;
        return false;
    }
}

void ConfigLoader::applyConfig(Mat3x4D& base_pos, Mat3x4D& kp, Mat3x4D& kd, 
                              Mat4x2F& tort_param, Mat4x4D& delta_param, Vec4D& phase_t) {
    // 应用基础位置参数
    for (int i = 0; i < 4; i++) {
        base_pos.col(i) = base_pos_config_.col(i);
    }
    
    // 应用PD参数
    for (int i = 0; i < 4; i++) {
        kp.col(i) = kp_config_.col(i);
        kd.col(i) = kd_config_.col(i);
    }
    
    // 应用TORT参数
    for (int i = 0; i < 4; i++) {
        tort_param.row(i) = tort_param_config_.row(i);
    }
    
    // 应用增量参数
    for (int i = 0; i < 4; i++) {
        delta_param.row(i) = delta_param_config_.row(i);
    }
    
    // 应用相位参数
    phase_t = phase_config_;
}

bool ConfigLoader::parseBasePosition(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("base_position:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section && line.find("leg") == 0 && line.find(":") != std::string::npos && line[0] != '#') {
            size_t colon_pos = line.find(":");
            std::string leg_name = trim(line.substr(0, colon_pos));
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            // 移除注释
            size_t comment_pos = values_str.find("#");
            if (comment_pos != std::string::npos) {
                values_str = trim(values_str.substr(0, comment_pos));
            }
            
            // 移除方括号
            if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                values_str = values_str.substr(1, values_str.length() - 2);
            }
            
            std::istringstream values_stream(values_str);
            std::string value;
            std::vector<double> values;
            
            while (std::getline(values_stream, value, ',')) {
                try {
                    values.push_back(std::stof(trim(value)));
                } catch (const std::exception& e) {
                    std::cout << "解析数值错误: " << trim(value) << std::endl;
                    return false;
                }
            }
            
            if (values.size() == 3) {
                int leg_index = -1;
                if (leg_name == "leg0") leg_index = 0;
                else if (leg_name == "leg1") leg_index = 1;
                else if (leg_name == "leg2") leg_index = 2;
                else if (leg_name == "leg3") leg_index = 3;
                
                if (leg_index >= 0) {
                    base_pos_config_.col(leg_index) << values[0], values[1], values[2];
                }
            }
        }
        
        // 检查是否离开当前section (遇到下一个section)
        if (in_section && line.find(":") != std::string::npos && 
            line.find("base_position:") == std::string::npos &&
            line.find("leg") != 0) {
            break;
        }
    }
    
    return true;
}

bool ConfigLoader::parseTortParams(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("tort_params:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section && line.find("leg") == 0 && line.find(":") != std::string::npos && line[0] != '#') {
            size_t colon_pos = line.find(":");
            std::string leg_name = trim(line.substr(0, colon_pos));
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                values_str = values_str.substr(1, values_str.length() - 2);
            }
            
            std::istringstream values_stream(values_str);
            std::string value;
            std::vector<double> values;
            
            while (std::getline(values_stream, value, ',')) {
                try {
                    values.push_back(std::stof(trim(value)));
                } catch (const std::exception& e) {
                    std::cout << "解析数值错误: " << trim(value) << std::endl;
                    return false;
                }
            }
            
            if (values.size() == 2) {
                int leg_index = -1;
                if (leg_name == "leg0") leg_index = 0;
                else if (leg_name == "leg1") leg_index = 1;
                else if (leg_name == "leg2") leg_index = 2;
                else if (leg_name == "leg3") leg_index = 3;
                
                if (leg_index >= 0) {
                    tort_param_config_.row(leg_index) << values[0], values[1];
                }
            }
        }
        
        // 检查是否离开当前section (遇到下一个section)
        if (in_section && line.find(":") != std::string::npos && 
            line.find("tort_params:") == std::string::npos &&
            line.find("leg") != 0) {
            break;
        }
    }
    
    return true;
}

bool ConfigLoader::parseDeltaParams(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("delta_params:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section && line.find("leg") == 0 && line.find(":") != std::string::npos && line[0] != '#') {
            size_t colon_pos = line.find(":");
            std::string leg_name = trim(line.substr(0, colon_pos));
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                values_str = values_str.substr(1, values_str.length() - 2);
            }
            
            std::istringstream values_stream(values_str);
            std::string value;
            std::vector<double> values;
            
            while (std::getline(values_stream, value, ',')) {
                try {
                    values.push_back(std::stof(trim(value)));
                } catch (const std::exception& e) {
                    std::cout << "解析数值错误: " << trim(value) << std::endl;
                    return false;
                }
            }
            
            if (values.size() == 4) {
                int leg_index = -1;
                if (leg_name == "leg0") leg_index = 0;
                else if (leg_name == "leg1") leg_index = 1;
                else if (leg_name == "leg2") leg_index = 2;
                else if (leg_name == "leg3") leg_index = 3;
                
                if (leg_index >= 0) {
                    delta_param_config_.row(leg_index) << values[0], values[1], values[2], values[3];
                }
            }
        }
        
        // 检查是否离开当前section (遇到下一个section)
        if (in_section && line.find(":") != std::string::npos && 
            line.find("delta_params:") == std::string::npos &&
            line.find("leg") != 0) {
            break;
        }
    }
    
    return true;
}

bool ConfigLoader::parseKpParams(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("kp_params:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section && line.find("leg") == 0 && line.find(":") != std::string::npos && line[0] != '#') {
            size_t colon_pos = line.find(":");
            std::string leg_name = trim(line.substr(0, colon_pos));
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                values_str = values_str.substr(1, values_str.length() - 2);
            }
            
            std::istringstream values_stream(values_str);
            std::string value;
            std::vector<double> values;
            
            while (std::getline(values_stream, value, ',')) {
                try {
                    values.push_back(std::stof(trim(value)));
                } catch (const std::exception& e) {
                    std::cout << "解析数值错误: " << trim(value) << std::endl;
                    return false;
                }
            }
            
            if (values.size() == 3) {
                int leg_index = -1;
                if (leg_name == "leg0") leg_index = 0;
                else if (leg_name == "leg1") leg_index = 1;
                else if (leg_name == "leg2") leg_index = 2;
                else if (leg_name == "leg3") leg_index = 3;
                
                if (leg_index >= 0) {
                    kp_config_.col(leg_index) << values[0], values[1], values[2];
                }
            }
        }
        
        // 检查是否离开当前section (遇到下一个section)
        if (in_section && line.find(":") != std::string::npos && 
            line.find("kp_params:") == std::string::npos &&
            line.find("leg") != 0) {
            break;
        }
    }
    
    return true;
}

bool ConfigLoader::parseKdParams(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("kd_params:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section && line.find("leg") == 0 && line.find(":") != std::string::npos && line[0] != '#') {
            size_t colon_pos = line.find(":");
            std::string leg_name = trim(line.substr(0, colon_pos));
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                values_str = values_str.substr(1, values_str.length() - 2);
            }
            
            std::istringstream values_stream(values_str);
            std::string value;
            std::vector<double> values;
            
            while (std::getline(values_stream, value, ',')) {
                try {
                    values.push_back(std::stof(trim(value)));
                } catch (const std::exception& e) {
                    std::cout << "解析数值错误: " << trim(value) << std::endl;
                    return false;
                }
            }
            
            if (values.size() == 3) {
                int leg_index = -1;
                if (leg_name == "leg0") leg_index = 0;
                else if (leg_name == "leg1") leg_index = 1;
                else if (leg_name == "leg2") leg_index = 2;
                else if (leg_name == "leg3") leg_index = 3;
                
                if (leg_index >= 0) {
                    kd_config_.col(leg_index) << values[0], values[1], values[2];
                }
            }
        }
        
        // 检查是否离开当前section (遇到下一个section)
        if (in_section && line.find(":") != std::string::npos && 
            line.find("kd_params:") == std::string::npos &&
            line.find("leg") != 0) {
            break;
        }
    }
    
    return true;
}

bool ConfigLoader::parsePhaseValues(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("phase_values:") != std::string::npos) {
            in_section = true;
            // 如果值在同一行，直接处理
            size_t colon_pos = line.find(":");
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            if (!values_str.empty()) {
                // 移除注释
                size_t comment_pos = values_str.find("#");
                if (comment_pos != std::string::npos) {
                    values_str = trim(values_str.substr(0, comment_pos));
                }
                
                // 移除方括号
                if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                    values_str = values_str.substr(1, values_str.length() - 2);
                }
                
                std::istringstream values_stream(values_str);
                std::string value;
                std::vector<double> values;
                
                while (std::getline(values_stream, value, ',')) {
                    try {
                        values.push_back(std::stof(trim(value)));
                    } catch (const std::exception& e) {
                        std::cout << "解析数值错误: " << trim(value) << std::endl;
                        return false;
                    }
                }
                
                if (values.size() == 4) {
                    phase_config_ << values[0], values[1], values[2], values[3];
                }
                return true;
            }
            continue;
        }
        
        if (in_section && line.find(":") != std::string::npos && line[0] != '#') {
            size_t colon_pos = line.find(":");
            std::string values_str = trim(line.substr(colon_pos + 1));
            
            // 移除注释
            size_t comment_pos = values_str.find("#");
            if (comment_pos != std::string::npos) {
                values_str = trim(values_str.substr(0, comment_pos));
            }
            
            // 移除方括号
            if (values_str[0] == '[' && values_str[values_str.length()-1] == ']') {
                values_str = values_str.substr(1, values_str.length() - 2);
            }
            
            std::istringstream values_stream(values_str);
            std::string value;
            std::vector<double> values;
            
            while (std::getline(values_stream, value, ',')) {
                try {
                    values.push_back(std::stof(trim(value)));
                } catch (const std::exception& e) {
                    std::cout << "解析数值错误: " << trim(value) << std::endl;
                    return false;
                }
            }
            
            if (values.size() == 4) {
                phase_config_ << values[0], values[1], values[2], values[3];
            }
            break;
        }
    }
    
    return true;
}

std::string ConfigLoader::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\n\r");
    return str.substr(first, (last - first + 1));
}

}
