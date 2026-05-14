#ifndef ENUM_TYPE_HPP
#define ENUM_TYPE_HPP
#define LEG_NUM 4


enum class StateName
{
    Invaild,
    Init,
    Deinit,
    Tort,
    Jump,
    FreeAngle,
    FixedStand,
    Turn,
    Count
};

enum class LogLevel
{
    TRACE = 0, // 追踪最详细的信息
    DEBUG = 1, // 调试信息
    INFO = 2,  // 一般信息
    WARN = 3,  // 警告信息
    ERROR = 4, // 错误信息
    FATAL = 5  // 致命错误
};

// 日志模块定义
enum Module
{
    MODULE_SYSTEM = 0,
    MODULE_FSM = 1,
    MODULE_HAND = 2,
    MODULE_MOTOR = 3,
    MODULE_MATH = 4,
    MODULE_SEQ = 5
};

enum class WaveState
{
    Stand,
    Wave,
    Wait,
    Straight,
    DynamicWave
};
enum class LegId
{
    RightFront = 0,
    LeftFront = 1,
    RightRear = 2,
    LeftRear = 3,
};
enum class LegSide : int
{
    Left = 1,
    Right = -1
};

enum class StateModeName
{
    Stand,
    StandMotion,
    WalkF,
    //    WalkL, // 暂时不用
    //    WalkR, // 暂时不用
    WalkB,
    TurnR,
    TurnL,
    MoveL,
    MoveR,
};

enum class FSMMode
{
    NORMAL,
    CHANGE,
};

enum class GaitType
{
    Stand,
    Tort,
    Walk,
    Straight
};

enum class GaitClacType
{
    DeltaClac,
    ConstCalc
};

enum class MotionState
{
    Swing,       // 摆动相
    Stance,      // 线性相
    PreStance,   // 线性相
    EndStance,   // 线性相
    Step,        // 步移相
    Angle,       // 角度相
    TransSwing,  // 过渡摆动相
    TransStance, // 过渡线性相
    TransStop,   // 过渡停止相
    Stop         // 停止
};

enum class DirMode
{
    Forward,
    Left,
    Right
};



// Unitree
enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};


#endif
