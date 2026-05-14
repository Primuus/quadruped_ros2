#pragma once

#include <iostream>
#include <cstdio>

// 编译时日志级别过滤
#define ANT_LOG_MIN_LEVEL 0 // INFO

// 简单的日志输出宏 - 直接输出到stdout，支持可选的模块前缀
#define LOG_TRACE(message, ...)             \
    do                                      \
    {                                       \
        if (ANT_LOG_MIN_LEVEL <= 0)         \
        {                                   \
            printf("[TRACE] ");             \
            printf(message, ##__VA_ARGS__); \
            printf("\n");                   \
            fflush(stdout);                 \
        }                                   \
    } while (0)

#define LOG_DEBUG(message, ...)             \
    do                                      \
    {                                       \
        if (ANT_LOG_MIN_LEVEL <= 1)         \
        {                                   \
            printf("[DEBUG] ");             \
            printf(message, ##__VA_ARGS__); \
            printf("\n");                   \
            fflush(stdout);                 \
        }                                   \
    } while (0)

#define LOG_INFO(message, ...)              \
    do                                      \
    {                                       \
        if (ANT_LOG_MIN_LEVEL <= 2)         \
        {                                   \
            printf("[INFO] ");              \
            printf(message, ##__VA_ARGS__); \
            printf("\n");                   \
            fflush(stdout);                 \
        }                                   \
    } while (0)

#define LOG_WARN(message, ...)              \
    do                                      \
    {                                       \
        if (ANT_LOG_MIN_LEVEL <= 3)         \
        {                                   \
            printf("[WARN] ");              \
            printf(message, ##__VA_ARGS__); \
            printf("\n");                   \
            fflush(stdout);                 \
        }                                   \
    } while (0)

#define LOG_ERROR(message, ...)             \
    do                                      \
    {                                       \
        if (ANT_LOG_MIN_LEVEL <= 4)         \
        {                                   \
            printf("[ERROR] ");             \
            printf(message, ##__VA_ARGS__); \
            printf("\n");                   \
            fflush(stdout);                 \
        }                                   \
    } while (0)

#define LOG_FATAL(message, ...)             \
    do                                      \
    {                                       \
        if (ANT_LOG_MIN_LEVEL <= 5)         \
        {                                   \
            printf("[FATAL] ");             \
            printf(message, ##__VA_ARGS__); \
            printf("\n");                   \
            fflush(stdout);                 \
        }                                   \
    } while (0)

void init_logging_system();
