#include "quadruped_config/free_angle_config.hpp"
#include <fstream>
#include <sstream>

namespace free_angle {

ConfigLoader::ConfigLoader(const std::string& config_file) 
    : config_file_(config_file) {
    // 初始化配置矩阵
    kp_config_.setZero();
    kd_config_.setZero();
    phase_config_.setZero();
    target_q_config_.setZero();
}

bool ConfigLoader::loadConfig() {
    std::ifstream file(config_file_);
    if (!file.is_open()) {
        std::cout << "无法打开配置文件: " << config_file_ << std::endl;
        return false;
    }
    
    std::string line;
    std::string content;
    
    while (std::getline(file, line)) {
        content += line + "\n";
    }
    file.close();
    
    // 解析各个部分
    bool success = true;
    success &= parseKpParams(content);
    success &= parseKdParams(content);
    success &= parsePhaseValues(content);
    success &= parseTargetQParams(content);

    if (success) {
        // std::cout << "配置文件解析成功: " << config_file_ << std::endl;
        return true;
    } else {
        std::cout << "配置文件解析失败: " << config_file_ << std::endl;
        return false;
    }
}

void ConfigLoader::applyConfig(Mat3x4D& kp, Mat3x4D& kd, Vec4D& phase_t, Mat3x4D& target_q) {
    // 应用PD参数
    for (int i = 0; i < 4; i++) {
        kp.col(i) = kp_config_.col(i);
        kd.col(i) = kd_config_.col(i);
    }
    
    // 应用相位参数
    phase_t = phase_config_;
    
    // 应用目标关节角度参数
    for (int i = 0; i < 4; i++) {
        target_q.col(i) = target_q_config_.col(i);
    }
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

bool ConfigLoader::parseTargetQParams(const std::string& content) {
    std::istringstream iss(content);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        line = trim(line);
        
        if (line.find("target_q_params:") != std::string::npos) {
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
                    target_q_config_.col(leg_index) << values[0], values[1], values[2];
                }
            }
        }
        
        // 检查是否离开当前section (遇到下一个section)
        if (in_section && line.find(":") != std::string::npos && 
            line.find("target_q_params:") == std::string::npos &&
            line.find("leg") != 0) {
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
