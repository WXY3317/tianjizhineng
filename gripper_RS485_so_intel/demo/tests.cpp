#include "controller.hpp"
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
            check(guarded.command({1, Action::Enable}).accepted, "guarded enable failed");
            check(guarded.command({2, Action::Move, 0.010, 0.01f}).accepted, "guarded move failed");
            h->state.motor_current_a = 0.249;
            check(!guarded.sample().current_limit_reached && h->holds == 0, "below threshold hold");
            h->state.motor_current_a = 0.25;
            const auto stopped = guarded.sample();
            check(stopped.current_limit_reached && stopped.holding_position && !stopped.at_target,
                  "threshold did not switch to hold");
            check(stopped.command_id == 2 && stopped.target_active && stopped.target_position_m == 0.050 &&
                  h->held_angle == 3.4 && h->holds == 1 && h->moves == 1, "hold capture incorrect");
            h->state.position_m = 0.049; h->state.motor_position_rad = 3.5; h->state.motor_current_a = 0.1;
            check(guarded.sample().target_position_m == 0.050 && h->holds == 1, "hold target drifted");
            check(guarded.command({3, Action::Move, 0.070, 0.01f}).accepted, "new move rejected");
            check(!guarded.sample().current_limit_reached, "new move failed to reset hold");
            h->state.motor_current_a = -0.25;
            check(guarded.sample().holding_position && h->holds == 2, "negative current ignored");
            check(guarded.command({4, Action::Disable}).accepted && !guarded.sample().holding_position,
                  "disable failed to cancel hold");
            h->state.motor_current_a = 0;
            check(guarded.command({5, Action::Enable}).accepted, "enable after disable failed");
            check(guarded.command({6, Action::Move, 0.010, 0.01f}).accepted, "retry move failed");
            h->throw_hold = true; h->state.motor_current_a = 0.3;
            check(!guarded.sample().valid && !h->state.enabled, "failed hold did not fail safe");
        }
        {
            auto reused = std::make_unique<Fake>();
            Controller c(std::move(reused)); c.init(true);
            check(c.sample().homed && !c.sample().enabled, "reuse zero must not enable");
            check(c.command({1, Action::Enable}).accepted, "confirmed existing zero cannot enable");
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
        check(!c.command({1, Action::Move, 0.06, 0.01f}).accepted, "unhomed movement accepted");
        check(hw->moves == 0, "unhomed command reached hardware");
        check(c.command({2, Action::Zero}).accepted, "zero failed");
        check(c.sample().homed && !c.sample().enabled, "zero must leave disabled");
        check(!c.command({3, Action::Move, 0.06, 0.01f}).accepted, "disabled movement accepted");
        check(c.command({4, Action::Enable}).accepted, "enable failed");
        check(!c.command({5, Action::Move, 0.099, 0.01f}).accepted, "range check failed");
        check(!c.command({6, Action::Move, 0.06, 0}).accepted, "zero speed accepted");
        check(!c.command({7, Action::Move, std::numeric_limits<double>::quiet_NaN(), 0.01f}).accepted, "NaN accepted");
        check(!c.command({8, Action::Move, 0.06, std::numeric_limits<float>::infinity()}).accepted, "infinity accepted");
        check(hw->moves == 0, "invalid movement reached hardware");
        check(c.command({9, Action::Move, 0.06, 0.01f}).accepted, "valid move failed");
        check(c.sample().at_target && c.sample().command_id == 9, "feedback target tracking failed");
        hw->connected = false;
        check(!c.sample().valid && !hw->state.enabled, "communication failure did not disable");
        hw->connected = true;
        check(!c.command({10, Action::Enable}).accepted, "communication failure must invalidate zero");
        check(c.command({11, Action::Zero}).accepted, "re-zero failed");
        check(c.command({12, Action::Enable}).accepted, "re-enable failed");
        hw->state.error_code = 4;
        check(c.sample().error_code == 4 && !hw->state.enabled, "fault failed to disable");
        check(c.command({13, Action::Clear}).accepted, "clear failed");
        hw->zero_ok = false;
        check(!c.command({14, Action::Zero}).accepted && !c.sample().homed, "failed zero accepted");
        hw->zero_ok = true;
        check(c.command({15, Action::Zero}).accepted, "zero failed");
        check(c.command({16, Action::Enable}).accepted, "enable failed");
        hw->throw_move = true;
        check(!c.command({17, Action::Move, 0.02, 0.01f}).accepted && !hw->state.enabled,
              "exception failed to disable");
        check(c.command({18, Action::Disable}).accepted, "disable failed");
        std::cout << "Controller state machine checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
