#ifndef FIXED_STAND_CONFIG_HPP
#define FIXED_STAND_CONFIG_HPP

#include "quadruped_common/eigen_type.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <filesystem>

namespace fixed_stand {
class ConfigLoader {
public:
    ConfigLoader(const std::string& config_file = "src/controller/quadruped/config/fix_stand_params.yaml");
    
    bool loadConfig();
    void applyConfig(Mat3x4D& kp, Mat3x4D& kd, Vec4D& phase_t, Mat3x4D& target_foot_pos);
    
private:
    std::string config_file_;
    Mat3x4D kp_config_;
    Mat3x4D kd_config_;
    Vec4D phase_config_;
    Mat3x4D target_foot_pos_config_;
    
    bool parseKpParams(const std::string& content);
    bool parseKdParams(const std::string& content);
    bool parsePhaseValues(const std::string& content);
    bool parseTargetFootPosParams(const std::string& content);
    
    std::string trim(const std::string& str);
};
}
#endif
