#pragma once

#include <Eigen/Dense>

// 夹爪运动学；公开接口统一使用 SI 单位（m）。
class grip_pawl
{
    public:
        grip_pawl();

        double fkine(double screw_pos) const;
        double ikine(double distance) const;
        // 闭合量对丝杠位移的导数；有效行程内为正。
        double get_closing_jac(double screw_pos) const;

    private:
        static constexpr double L_AB_ = 13.0;
        static constexpr double L_BC_ = 22.0;

        Eigen::Vector2d A_zeros_;
        Eigen::Vector2d C_;
        Eigen::Vector2d E_;
        Eigen::Vector2d CB_vec_zeros_;
        Eigen::Vector2d EF_vec_;
        Eigen::Vector2d friction_plate_;

        static Eigen::Matrix2d rot2(double theta);
};
