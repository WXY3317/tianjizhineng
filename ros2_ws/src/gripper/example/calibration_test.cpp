#include "calibration.hpp"
#include <iostream>
int main() {
    try {
        gripper_demo::Calibration c(DEMO_CALIBRATION_PATH);
        const double pi = std::acos(-1.0);
        if (std::abs(c.width(0)-0.098) > 1e-10 || std::abs(c.angle(0)-3507.0337260275*pi/180) > 1e-8)
            throw std::runtime_error("endpoint calibration failed");
        for (int i = 0; i <= 1000; ++i) {
            double width = 0.098*i/1000;
            if (std::abs(c.width(c.angle(width))-width) > 1e-10)
                throw std::runtime_error("command/feedback inverse failed");
            if (!std::isfinite(c.angular_speed(width, 0.01)) || c.angular_speed(width, 0.01) <= 0)
                throw std::runtime_error("speed conversion failed");
        }
        bool rejected = false;
        try { c.angle(-0.001); } catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("negative command accepted");
        rejected = false;
        try { c.angle(0.099); } catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("command outside stroke accepted");
        const double closed_slope = (c.width(c.closed_angle())-c.width(c.closed_angle()-0.001))/0.001;
        c.width(c.closed_angle()+0.0001/std::abs(closed_slope));
        rejected = false;
        try { c.width(c.closed_angle()+0.0003/std::abs(closed_slope)); }
        catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("feedback outside 0.2 mm tolerance accepted");
        std::cout << "Calibration endpoints and 1001 inverse mappings passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
