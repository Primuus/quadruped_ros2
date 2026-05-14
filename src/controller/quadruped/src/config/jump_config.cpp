#include "quadruped_config/jump_config.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace jump {

ConfigLoader::ConfigLoader(const std::string& config_file) 
    : config_file_(config_file) {
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
    
    stages_.clear();
    
    bool success = parseJumpSequence(content);
    
    if (success && !stages_.empty()) {
        // std::cout << "Jump配置加载成功, 共 " << stages_.size() << " 个阶段" << std::endl;
        return true;
    } else {
        std::cout << "Jump配置加载失败" << std::endl;
        return false;
    }
}

const std::vector<JumpStage>& ConfigLoader::getStages() const {
    return stages_;
}

int ConfigLoader::getStageCount() const {
    return stages_.size();
}

const JumpStage& ConfigLoader::getStage(int index) const {
    return stages_[index];
}


bool ConfigLoader::parseJumpSequence(const std::string& content) {
    // 查找 jump_sequence: 部分
    size_t seq_pos = content.find("jump_sequence:");
    if (seq_pos == std::string::npos) {
        std::cout << "未找到 jump_sequence 配置" << std::endl;
        return false;
    }
    
    // 找到 jump_sequence 的结束位置（下一个顶级key或文件结束）
    size_t seq_end = content.find("\ncommon:", seq_pos);
    if (seq_end == std::string::npos) {
        seq_end = content.length();
    }
    
    std::string seq_content = content.substr(seq_pos, seq_end - seq_pos);
    
    // 解析每个阶段块（以 "- stage:" 开头）
    std::istringstream iss(seq_content);
    std::string line;
    std::string current_block;
    bool in_stage = false;
    
    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);
        
        if (trimmed.find("- stage:") == 0) {
            // 保存上一个阶段
            if (!current_block.empty() && in_stage) {
                JumpStage stage;
                if (parseStageBlock(current_block, stage)) {
                    stages_.push_back(stage);
                }
            }
            current_block = line + "\n";
            in_stage = true;
        } else if (in_stage) {
            // 检查是否遇到新的顶级key（非缩进的行，且不是列表项）
            if (!line.empty() && line[0] != ' ' && line[0] != '\t' && 
                trimmed.find("- stage:") != 0 && !trimmed.empty()) {
                // 阶段结束
                if (!current_block.empty()) {
                    JumpStage stage;
                    if (parseStageBlock(current_block, stage)) {
                        stages_.push_back(stage);
                    }
                }
                current_block.clear();
                in_stage = false;
            } else {
                current_block += line + "\n";
            }
        }
    }
    
    // 处理最后一个阶段
    if (!current_block.empty() && in_stage) {
        JumpStage stage;
        if (parseStageBlock(current_block, stage)) {
            stages_.push_back(stage);
        }
    }
    
    return !stages_.empty();
}

bool ConfigLoader::parseStageBlock(const std::string& block, JumpStage& stage) {
    stage.base_pos.setZero();
    stage.kp.setZero();
    stage.kd.setZero();
    stage.phase.setZero();
    
    std::istringstream iss(block);
    std::string line;
    
    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);
        
        // 解析阶段名称 (格式: "- stage: "name"" 或 "stage: "name"")
        if (trimmed.find("- stage:") == 0 || trimmed.find("stage:") == 0) {
            size_t colon_pos = trimmed.find(":");
            stage.name = trim(trimmed.substr(colon_pos + 1));
            // 移除引号
            if (!stage.name.empty() && stage.name[0] == '"') {
                stage.name = stage.name.substr(1);
            }
            if (!stage.name.empty() && stage.name.back() == '"') {
                stage.name = stage.name.substr(0, stage.name.length() - 1);
            }
        }
    }
    
    // 解析位置和PD参数
    parseBasePosition(block, stage.base_pos);
    parseKpParams(block, stage.kp);
    parseKdParams(block, stage.kd);
    parsePhaseValues(block, stage.phase);
    
    return true;
}

bool ConfigLoader::parseBasePosition(const std::string& block, Mat3x4D& base_pos) {
    std::istringstream iss(block);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);
        
        if (trimmed.find("base_position:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section) {
            if (trimmed.find("leg") == 0 && trimmed.find(":") != std::string::npos) {
                size_t colon_pos = trimmed.find(":");
                std::string leg_name = trim(trimmed.substr(0, colon_pos));
                std::string values_str = trim(trimmed.substr(colon_pos + 1));
                
                std::vector<double> values = parseArray(values_str);
                int leg_idx = getLegIndex(leg_name);
                
                if (leg_idx >= 0 && values.size() == 3) {
                    base_pos.col(leg_idx) << values[0], values[1], values[2];
                }
            } else if (trimmed.find(":") != std::string::npos && 
                       trimmed.find("base_position:") == std::string::npos &&
                       trimmed.find("leg") != 0) {
                break;
            }
        }
    }
    
    return true;
}

bool ConfigLoader::parseKpParams(const std::string& block, Mat3x4D& kp) {
    std::istringstream iss(block);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);
        
        if (trimmed.find("kp_params:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section) {
            if (trimmed.find("leg") == 0 && trimmed.find(":") != std::string::npos) {
                size_t colon_pos = trimmed.find(":");
                std::string leg_name = trim(trimmed.substr(0, colon_pos));
                std::string values_str = trim(trimmed.substr(colon_pos + 1));
                
                std::vector<double> values = parseArray(values_str);
                int leg_idx = getLegIndex(leg_name);
                
                if (leg_idx >= 0 && values.size() == 3) {
                    kp.col(leg_idx) << values[0], values[1], values[2];
                }
            } else if (trimmed.find(":") != std::string::npos && 
                       trimmed.find("kp_params:") == std::string::npos &&
                       trimmed.find("leg") != 0) {
                break;
            }
        }
    }
    
    return true;
}

bool ConfigLoader::parseKdParams(const std::string& block, Mat3x4D& kd) {
    std::istringstream iss(block);
    std::string line;
    bool in_section = false;
    
    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);
        
        if (trimmed.find("kd_params:") != std::string::npos) {
            in_section = true;
            continue;
        }
        
        if (in_section) {
            if (trimmed.find("leg") == 0 && trimmed.find(":") != std::string::npos) {
                size_t colon_pos = trimmed.find(":");
                std::string leg_name = trim(trimmed.substr(0, colon_pos));
                std::string values_str = trim(trimmed.substr(colon_pos + 1));
                
                std::vector<double> values = parseArray(values_str);
                int leg_idx = getLegIndex(leg_name);
                
                if (leg_idx >= 0 && values.size() == 3) {
                    kd.col(leg_idx) << values[0], values[1], values[2];
                }
            } else if (trimmed.find(":") != std::string::npos && 
                       trimmed.find("kd_params:") == std::string::npos &&
                       trimmed.find("leg") != 0) {
                break;
            }
        }
    }
    
    return true;
}

bool ConfigLoader::parsePhaseValues(const std::string& block, Vec4D& phase) {
    std::istringstream iss(block);
    std::string line;
    
    while (std::getline(iss, line)) {
        std::string trimmed = trim(line);
        
        if (trimmed.find("phase_values:") != std::string::npos) {
            size_t colon_pos = trimmed.find(":");
            std::string values_str = trim(trimmed.substr(colon_pos + 1));
            
            std::vector<double> values = parseArray(values_str);
            if (values.size() == 4) {
                phase << values[0], values[1], values[2], values[3];
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

std::vector<double> ConfigLoader::parseArray(const std::string& str) {
    std::vector<double> values;
    std::string s = trim(str);
    
    // 移除注释
    size_t comment_pos = s.find("#");
    if (comment_pos != std::string::npos) {
        s = trim(s.substr(0, comment_pos));
    }
    
    // 移除方括号
    if (s.length() >= 2 && s[0] == '[' && s[s.length()-1] == ']') {
        s = s.substr(1, s.length() - 2);
    }
    
    std::istringstream iss(s);
    std::string item;
    
    while (std::getline(iss, item, ',')) {
        try {
            values.push_back(std::stof(trim(item)));
        } catch (...) {
            // 忽略解析错误
        }
    }
    
    return values;
}

int ConfigLoader::getLegIndex(const std::string& leg_name) {
    if (leg_name == "leg0") return 0;
    if (leg_name == "leg1") return 1;
    if (leg_name == "leg2") return 2;
    if (leg_name == "leg3") return 3;
    return -1;
}

}
