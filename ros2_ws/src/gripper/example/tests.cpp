#include "../src/controller.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gripper_demo;
class Fake final : public Hardware {
public:
    HardwareState state{0.085, 0, 0, 0, false};
    bool connected{true}, zero_ok{true}, throw_move{false}, throw_hold{false}, follow_moves{true};
    int holds{0}; double held_angle{0};
    int moves{0}, shutdowns{0};
    void init() override {}
    bool read(HardwareState& out) override { out = state; return connected; }
    bool zero() override { return zero_ok; }
    void enable_position() override { state.enabled = true; }
    void disable() override { state.enabled = false; }
    void clear() override { state.error_code = 0; }
    void move(double p, float) override {
        if (throw_move) throw std::runtime_error("transport failure");
        ++moves; if (follow_moves) state.position_m = p;
    }
    void hold_position(double angle, float) override {
        if (throw_hold) throw std::runtime_error("hold transport failure");
        ++holds; held_angle = angle;
    }
    void shutdown() noexcept override { ++shutdowns; state.enabled = false; }
};
void check(bool passed, const char* what) { if (!passed) throw std::runtime_error(what); }
int main() {
    try {
        for (double threshold : {0.0, -0.1, std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::quiet_NaN()}) {
            bool rejected = false;
            try { Controller invalid(std::make_unique<Fake>(), threshold); }
            catch (const std::invalid_argument&) { rejected = true; }
            check(rejected, "invalid threshold accepted");
        }
        {
            auto hardware = std::make_unique<Fake>(); auto* h = hardware.get();
            h->follow_moves = false; h->state.position_m = 0.050; h->state.motor_position_rad = 3.4;
            Controller guarded(std::move(hardware), 0.25); guarded.init(true);
            check(guarded.command({Action::Enable}).accepted, "guarded enable failed");
            check(guarded.command({Action::Move, 0.010, 0.01f}).accepted, "guarded move failed");
            h->state.motor_current_a = 0.249;
            check(!guarded.sample().current_limit_reached && h->holds == 0, "below threshold hold");
            h->state.motor_current_a = 0.25;
            const auto stopped = guarded.sample();
            check(stopped.current_limit_reached && stopped.holding_position && !stopped.at_target,
                  "threshold did not switch to hold");
            check(stopped.target_active && stopped.target_position_m == 0.050 &&
                  h->held_angle == 3.4 && h->holds == 1 && h->moves == 1, "hold capture incorrect");
            h->state.position_m = 0.049; h->state.motor_position_rad = 3.5; h->state.motor_current_a = 0.1;
            check(guarded.sample().target_position_m == 0.050 && h->holds == 1, "hold target drifted");
            check(guarded.command({Action::Move, 0.070, 0.01f}).accepted, "new move rejected");
            check(!guarded.sample().current_limit_reached, "new move failed to reset hold");
            h->state.motor_current_a = -0.25;
            check(guarded.sample().holding_position && h->holds == 2, "negative current ignored");
            check(guarded.command({Action::Disable}).accepted && !guarded.sample().holding_position,
                  "disable failed to cancel hold");
            h->state.motor_current_a = 0;
            check(guarded.command({Action::Enable}).accepted, "enable after disable failed");
            check(guarded.command({Action::Move, 0.010, 0.01f}).accepted, "retry move failed");
            h->throw_hold = true; h->state.motor_current_a = 0.3;
            check(!guarded.sample().valid && !h->state.enabled, "failed hold did not fail safe");
        }
        {
            auto reused = std::make_unique<Fake>();
            Controller c(std::move(reused)); c.init(true);
            check(c.sample().homed && !c.sample().enabled, "reuse zero must not enable");
            check(c.command({Action::Enable}).accepted, "confirmed existing zero cannot enable");
        }
        {
            auto bad = std::make_unique<Fake>(); bad->connected = false;
            Controller c(std::move(bad));
            bool rejected = false;
            try { c.init(true); } catch (const std::runtime_error&) { rejected = true; }
            check(rejected, "existing zero accepted without communication");
        }
        auto fake = std::make_unique<Fake>(); auto* hw = fake.get();
        Controller c(std::move(fake)); c.init();
        check(!c.command({Action::Move, 0.06, 0.01f}).accepted, "unhomed movement accepted");
        check(hw->moves == 0, "unhomed command reached hardware");
        check(c.command({Action::Zero}).accepted, "zero failed");
        check(c.sample().homed && !c.sample().enabled, "zero must leave disabled");
        check(!c.command({Action::Move, 0.06, 0.01f}).accepted, "disabled movement accepted");
        check(c.command({Action::Enable}).accepted, "enable failed");
        check(!c.command({Action::Move, 0.099, 0.01f}).accepted, "range check failed");
        check(!c.command({Action::Move, 0.06, 0}).accepted, "zero speed accepted");
        check(!c.command({Action::Move, std::numeric_limits<double>::quiet_NaN(), 0.01f}).accepted, "NaN accepted");
        check(!c.command({Action::Move, 0.06, std::numeric_limits<float>::infinity()}).accepted, "infinity accepted");
        check(hw->moves == 0, "invalid movement reached hardware");
        check(c.command({Action::Move, 0.06, 0.01f}).accepted, "valid move failed");
        check(c.sample().at_target, "feedback target tracking failed");
        hw->connected = false;
        check(!c.sample().valid && !hw->state.enabled, "communication failure did not disable");
        hw->connected = true;
        check(!c.command({Action::Enable}).accepted, "communication failure must invalidate zero");
        check(c.command({Action::Zero}).accepted, "re-zero failed");
        check(c.command({Action::Enable}).accepted, "re-enable failed");
        hw->state.error_code = 4;
        check(c.sample().error_code == 4 && !hw->state.enabled, "fault failed to disable");
        check(c.command({Action::Clear}).accepted, "clear failed");
        hw->zero_ok = false;
        check(!c.command({Action::Zero}).accepted && !c.sample().homed, "failed zero accepted");
        hw->zero_ok = true;
        check(c.command({Action::Zero}).accepted, "zero failed");
        check(c.command({Action::Enable}).accepted, "enable failed");
        hw->throw_move = true;
        check(!c.command({Action::Move, 0.02, 0.01f}).accepted && !hw->state.enabled,
              "exception failed to disable");
        check(c.command({Action::Disable}).accepted, "disable failed");
        {
            auto l = std::make_unique<Fake>(); auto* left_hw = l.get();
            auto r = std::make_unique<Fake>(); auto* right_hw = r.get();
            Controller left(std::move(l)), right(std::move(r));
            left.init(true); right.init(true);
            check(left.command({Action::Enable}).accepted && right.command({Action::Enable}).accepted,
                  "dual enable failed");
            check(left.command({Action::Move, 0.06, 0.01f}).accepted &&
                  right.command({Action::Move, 0.03, 0.01f}).accepted, "dual move failed");
            left_hw->state.motor_current_a = 0.6;
            check(left.sample().holding_position && !right.sample().holding_position && right_hw->holds == 0,
                  "left current hold affected right");
            check(right.sample().target_position_m == 0.03, "right target overwritten");
            left_hw->connected = false;
            check(!left.sample().valid && right.sample().valid && right.sample().enabled,
                  "left fault affected independent right controller");
        }
        std::cout << "Controller state machine checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
