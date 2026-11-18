#include "calibration.hpp"
#include <iostream>
#include <utility>
int main() {
    try {
        gripper_demo::Calibration c(DEMO_CALIBRATION_PATH);
        if (std::abs(c.width(0)-EXPECTED_OPEN_WIDTH) > 1e-10 || std::abs(c.angle(0)-EXPECTED_CLOSED_ANGLE) > 1e-8)
            throw std::runtime_error("endpoint calibration failed");
#ifdef EXPECTED_MEASURED_ANGLE
        if (std::abs(c.width(EXPECTED_MEASURED_ANGLE)-EXPECTED_MEASURED_WIDTH) > 0.0001)
            throw std::runtime_error("measured intermediate width mismatch");
#endif
#ifdef EXPECTED_MEASURED_ANGLE_2
        if (std::abs(c.width(EXPECTED_MEASURED_ANGLE_2)-EXPECTED_MEASURED_WIDTH_2) > 0.0001)
            throw std::runtime_error("second measured intermediate width mismatch");
#endif
#ifdef EXPECTED_MEASURED_ANGLE_3
        if (std::abs(c.width(EXPECTED_MEASURED_ANGLE_3)-EXPECTED_MEASURED_WIDTH_3) > 0.0001)
            throw std::runtime_error("third measured intermediate width mismatch");
#endif
#ifdef RIGHT_MEASUREMENTS
        for (const auto& point : {std::pair<double, double>{46.6546821594, 0.013},
                                  {38.7319602966, 0.025}, {23.5281009674, 0.050},
                                  {13.5884809494, 0.066}}) {
            if (std::abs(c.width(point.first)-point.second) > 0.0001)
                throw std::runtime_error("right measured width mismatch");
        }
#endif
        for (int i = 0; i <= 1000; ++i) {
            double width = EXPECTED_OPEN_WIDTH*i/1000;
            if (std::abs(c.width(c.angle(width))-width) > 1e-10)
                throw std::runtime_error("command/feedback inverse failed");
            if (!std::isfinite(c.angular_speed(width, 0.01)) || c.angular_speed(width, 0.01) <= 0)
                throw std::runtime_error("speed conversion failed");
        }
        bool rejected = false;
        try { c.angle(-0.001); } catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("negative command accepted");
        rejected = false;
        try { c.angle(EXPECTED_OPEN_WIDTH+0.001); } catch (const std::invalid_argument&) { rejected = true; }
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
