#pragma once

#include <Eigen/Dense>

//夹爪
class grip_pawl
{
    private:
        Eigen::Matrix3d rotz(double angle);
        double R;
        Eigen::Vector2d F;
        Eigen::Vector3d FC_zeros;
        Eigen::Vector2d CE_vec;
        int jac_sign;
        double L_zeros_offset;

    public:
        grip_pawl(double R, Eigen::Vector2d F, Eigen::Vector3d FC_zeros, double fric_plate_thickness, Eigen::Vector2d CE_vec, double L_zeros_offset, int jac_sign);
        double fkine(double pos);
        double ikine(double pos);
        double get_jac(double pos);
};
