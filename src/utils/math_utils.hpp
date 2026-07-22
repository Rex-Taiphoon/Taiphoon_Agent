/**
 * @file math_utils.hpp
 * @brief 數學工具函式，提供浮點數比較與數值合法性檢查
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#pragma once

#include <cmath>
#include <limits>

namespace taiphoon
{
namespace utils
{

constexpr double DEFAULT_EPSILON = 1e-6;

inline bool almost_equal(double a, double b, double epsilon = DEFAULT_EPSILON)
{
    return std::abs(a - b) < epsilon;
}

inline bool is_valid_float(float value)
{
    return !std::isnan(value) && !std::isinf(value);
}

inline bool is_valid_double(double value)
{
    return !std::isnan(value) && !std::isinf(value);
}

}  // namespace utils
}  // namespace taiphoon
