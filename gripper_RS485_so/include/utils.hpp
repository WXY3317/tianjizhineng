#pragma once

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

// ─────────────────── 常量 ────────────────────

inline constexpr double pad_thickness = 8.0;   // 软垫总厚度（mm）

// ──────────────────── 时间工具 ────────────────────

/**
 * @brief 获取当前时间戳字符串，格式 HH:MM:SS.mmm
 */
inline std::string now_str() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << std::put_time(&bt, "%H:%M:%S") << '.'
        << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

/**
 * @brief 获取文件名用时间戳字符串，格式 YYYYMMDD_HHMMSS
 */
inline std::string now_filename() {
    auto now = std::chrono::system_clock::now();
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ──────────────────── 日志目录 ────────────────────

/**
 * @brief 创建 log 目录（如不存在）
 */
inline void create_log_dir() {
    std::filesystem::create_directories("log");
}

// ──────────────────── 断言工具 ────────────────────

/**
 * @brief 比较两个浮点数是否接近，打印 [PASS]/[FAIL] 结果
 *
 * @param label    测试描述
 * @param actual   实际值
 * @param expected 期望值
 * @param tol      容差（默认 1e-3）
 * @return true 如果在容差范围内
 */
inline bool expect_near(const std::string &label, double actual,
                        double expected, double tol = 1e-3) {
    bool ok = std::fabs(actual - expected) < tol;
    if (ok) {
        std::cout << "[PASS] " << label << ": " << actual
                  << " (expected " << expected << ")" << std::endl;
    } else {
        std::cout << "[FAIL] " << label << ": " << actual
                  << " (expected " << expected << ", tol=" << tol << ")"
                  << std::endl;
    }
    return ok;
}
