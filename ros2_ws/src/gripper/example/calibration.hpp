#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gripper_demo {
// Endpoint normalization of the original nonlinear model. Interior accuracy needs measurement.
class Calibration {
public:
    struct Point { double angle_rad, width_m, force_per_percent; };
    explicit Calibration(const std::string& path) {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("cannot open calibration: " + path);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream input(line); Point p{}; std::string extra;
            if (!(input >> p.angle_rad >> p.width_m >> p.force_per_percent) || (input >> extra) ||
                !std::isfinite(p.angle_rad) || !std::isfinite(p.width_m) ||
                !std::isfinite(p.force_per_percent) || p.force_per_percent <= 0)
                throw std::runtime_error("invalid calibration row");
            if (!points_.empty() && (p.angle_rad <= points_.back().angle_rad || p.width_m >= points_.back().width_m))
                throw std::runtime_error("calibration must have increasing angle and decreasing width");
            points_.push_back(p);
        }
        if (points_.size() < 3 || std::abs(points_.front().angle_rad) > 1e-9 ||
            std::abs(points_.front().width_m-0.098) > 1e-9 || std::abs(points_.back().width_m) > 1e-9)
            throw std::runtime_error("calibration endpoints must be zero angle / 98 mm and closed / 0 mm");
    }
    double angle(double width) const {
        if (!std::isfinite(width) || width < 0 || width > 0.098)
            throw std::invalid_argument("width outside measured range");
        const auto i = width_segment(width);
        return interpolate(width, points_[i].width_m, points_[i+1].width_m, points_[i].angle_rad, points_[i+1].angle_rad);
    }
    double width(double angle) const {
        check_angle(angle); const auto i = angle_segment(angle);
        return interpolate(angle, points_[i].angle_rad, points_[i+1].angle_rad, points_[i].width_m, points_[i+1].width_m);
    }
    double angular_speed(double width, double speed) const {
        const auto i = width_segment(width);
        return speed * std::abs((points_[i+1].angle_rad-points_[i].angle_rad)/(points_[i+1].width_m-points_[i].width_m));
    }
    double estimated_force(double angle, double percent) const {
        check_angle(angle); const auto i = angle_segment(angle);
        return percent * interpolate(angle, points_[i].angle_rad, points_[i+1].angle_rad, points_[i].force_per_percent, points_[i+1].force_per_percent);
    }
    double closed_angle() const { return points_.back().angle_rad; }
private:
    static double interpolate(double x, double a, double b, double va, double vb) { return va+(x-a)*(vb-va)/(b-a); }
    void check_angle(double angle) const {
        if (!std::isfinite(angle)) throw std::runtime_error("non-finite motor angle");
        const auto i = angle_segment(angle);
        const double measured_width = interpolate(angle, points_[i].angle_rad, points_[i+1].angle_rad,
                                                   points_[i].width_m, points_[i+1].width_m);
        // Feedback tolerance in physical units, independent of varying mechanism Jacobian.
        constexpr double tolerance_m = 0.0002;
        if (measured_width < -tolerance_m || measured_width > 0.098+tolerance_m)
            throw std::runtime_error("feedback outside calibrated stroke: angle_rad="+std::to_string(angle)+
                                     " width_m="+std::to_string(measured_width));
    }
    std::size_t angle_segment(double angle) const {
        auto it = std::upper_bound(points_.begin(), points_.end(), angle, [](double v, const Point& p) { return v < p.angle_rad; });
        return static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(it-points_.begin()-1, 0, points_.size()-2));
    }
    std::size_t width_segment(double width) const {
        auto it = std::upper_bound(points_.begin(), points_.end(), width, [](double v, const Point& p) { return v > p.width_m; });
        return static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(it-points_.begin()-1, 0, points_.size()-2));
    }
    std::vector<Point> points_;
};
} // namespace gripper_demo
