#ifndef TURN_CONFIG_HPP
#define TURN_CONFIG_HPP

#include "quadruped_common/eigen_type.hpp"
#include <string>
#include <iostream>
#include <vector>
#include <filesystem>

namespace turn {
class ConfigLoader {
public:
    ConfigLoader(const std::string& config_file = "src/controller/quadruped/config/jump_params.yaml");
    
    bool loadConfig();
    void applyConfig(Mat3x4D& base_pos, Mat3x4D& kp, Mat3x4D& kd, 
                    Mat4x2F& tort_param, Mat4x4D& delta_param, Vec4D& phase_t);
    
private:
    std::string config_file_;
    Mat3x4D base_pos_config_;
    Mat3x4D kp_config_;
    Mat3x4D kd_config_;
    Mat4x2F tort_param_config_;
    Mat4x4D delta_param_config_;
    Vec4D phase_config_;
    
    bool parseBasePosition(const std::string& content);
    bool parseKpParams(const std::string& content);
    bool parseKdParams(const std::string& content);
    bool parseTortParams(const std::string& content);
    bool parseDeltaParams(const std::string& content);
    bool parsePhaseValues(const std::string& content);
    
    std::string trim(const std::string& str);
};
}
#endif
