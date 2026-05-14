#ifndef MATH_ANT_HPP
#define MATH_ANT_HPP
#include "quadruped_common/eigen_type.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <type_traits>

#define PI 3.1415926f

namespace math
{
    // 平方缩放
    template <typename T>
    void ScaleNumbers(T *a, T *b, double n)
    {
        // 计算缩放因子
        double scaleFactor = std::sqrt(n);
        // 直接修改指针所指向的值
        *a = static_cast<T>(*a * scaleFactor);
        *b = static_cast<T>(*b * scaleFactor);
    }
    // 确保输入值在[min, max]范围内
    template <typename T>
    T ZoneLimit(T *num, T min, T max)
    {
        *num = std::clamp(*num, min, max);
        return *num;
    }
    // 确保输入值在[-max, max]范围内
    template <typename T>
    T Limit(T *num, T max)
    {
        *num = std::clamp(*num, -max, max);
        return *num;
    }
    // 绝对值
    template <typename T>
    T Abs(const T &num)
    {
        return std::abs(num);
    }
    // 浮点数转无符号整数
    template <typename T, typename U = uint32_t>
    U FloatToUint(T x, T x_min, T x_max, int bits)
    {
        static_assert(std::is_floating_point<T>::value, "T必须是浮点类型");
        static_assert(std::is_unsigned<U>::value, "U必须是无符号整数类型");
        T span = x_max - x_min;
        x = std::clamp(x, x_min, x_max);
        return static_cast<U>((x - x_min) / span * ((1ULL << bits) - 1));
    }
    template <typename T1, typename T2>
    inline T1 max(const T1 a, const T2 b)
    {
        return (a > b ? a : b);
    }
    template <typename T1, typename T2>
    inline T1 min(const T1 a, const T2 b)
    {
        return (a < b ? a : b);
    }
    template <typename T>
    inline T Saturation(const T a, Vec2D limits)
    {
        T lowLim, highLim;
        if (limits(0) > limits(1))
        {
            lowLim = limits(1);
            highLim = limits(0);
        }
        else
        {
            lowLim = limits(0);
            highLim = limits(1);
        }
        if (a < lowLim)
        {
            return lowLim;
        }
        else if (a > highLim)
        {
            return highLim;
        }
        else
        {
            return a;
        }
    }

    // 平方和
    template <typename T>
    T SquareSum(T x, T y)
    {
        return x * x + y * y;
    }
    // 平方差
    template <typename T>
    T SquareDiff(T x, T y)
    {
        return x * x - y * y;
    }
    // 平方
    template <typename T>
    T Square(T x)
    {
        return x * x;
    }
    // 二维距离
    template <typename T>
    T Distance2D(T x1, T y1, T x2, T y2)
    {
        return sqrt(Square((x1 - x2)) + Square((y1 - y2)));
    }
    // 角度转弧度
    template <typename T>
    T AngleToRadian(T angle)
    {
        return angle * PI / 180.0f;
    }
    // 弧度转角度
    template <typename T>
    T RadianToAngle(T radian)
    {
        return radian * 180.0f / PI;
    }
    template <typename T>
    T LimitQ1(T q)
    {
        if (q > (110.0f * PI / 180.0f))
        {
            q = 110.0f * PI / 180.0f;
        }
        else if (q < (-110.0f * PI / 180.0f))
        {
            q = -110.0f * PI / 180.0f;
        }
        return q;
    }
    template <typename T>
    T LimitQ2(T q)
    {
        if (q > (160.0f * PI / 180.0f))
        {
            q = 160.0f * PI / 180.0f;
        }
        else if (q < (-80.0f * PI / 180.0f))
        {
            q = -80.0f * PI / 180.0f;
        }
        return q;
    }
    template <typename T>
    T LimitQ3(T q)
    {
        if (q <= (0.0f * PI / 180.0f))
        {
            q = 0.0f * PI / 180.0f;
        }
        else if (q > (130.0f * PI / 180.0f))
        {
            q = 130.0f * PI / 180.0f;
        }
        return q;
    }
    template <typename T>
    void LimitTime(T &t)
    {
        if (t < 0.0f)
        {
            t = 0.001f;
        }
    }
} // namespace math
#endif