#ifndef JUMP_CONFIG_HPP
#define JUMP_CONFIG_HPP

#include "quadruped_common/eigen_type.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <filesystem>

namespace jump {

// 单个阶段配置
struct JumpStage {
    std::string name;
    Mat3x4D base_pos;
    Mat3x4D kp;
    Mat3x4D kd;
    Vec4D phase;
};

class ConfigLoader {
public:
    ConfigLoader(const std::string& config_file = "src/controller/quadruped/config/jump_params.yaml");
    
    bool loadConfig();
    
    // 获取所有阶段（按顺序）
    const std::vector<JumpStage>& getStages() const;
    
    // 获取阶段数量
    int getStageCount() const;
    
    // 获取指定阶段
    const JumpStage& getStage(int index) const;

    
private:
    std::string config_file_;
    std::vector<JumpStage> stages_;
    // 解析方法
    bool parseJumpSequence(const std::string& content);
    bool parseStageBlock(const std::string& block, JumpStage& stage);
    bool parseBasePosition(const std::string& block, Mat3x4D& base_pos);
    bool parseKpParams(const std::string& block, Mat3x4D& kp);
    bool parseKdParams(const std::string& block, Mat3x4D& kd);
    bool parsePhaseValues(const std::string& block, Vec4D& phase);
    
    std::string trim(const std::string& str);
    std::vector<double> parseArray(const std::string& str);
    int getLegIndex(const std::string& leg_name);
};

}

#endif
