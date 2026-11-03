#include "controller.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <vector>

namespace {
volatile std::sig_atomic_t stopped = 0;
void signal_handler(int) { stopped = 1; }
std::string json_string(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : text) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32 || c >= 127) out << "\\u00" << std::hex << std::setw(2)
                                       << std::setfill('0') << static_cast<int>(c);
        else out << c;
    }
    out << '"'; return out.str();
}
void feedback(std::ostream& out, const gripper_demo::Feedback& f) {
    out << std::boolalpha << std::setprecision(9)
        << "{\"type\":\"feedback\",\"stamp_ns\":" << f.stamp_ns
        << ",\"valid\":" << f.valid;
    if (f.valid) out << ",\"position_m\":" << f.position_m
        << ",\"motor_position_rad\":" << f.motor_position_rad
        << ",\"motor_current_a\":" << f.motor_current_a
        << ",\"motor_torque_percent\":" << f.motor_torque_percent
        << ",\"force_n\":" << f.force_n << ",\"error_code\":" << f.error_code
        << ",\"enabled\":" << f.enabled;
    else out << ",\"position_m\":null,\"motor_position_rad\":null,\"motor_current_a\":null,\"motor_torque_percent\":null,\"force_n\":null,"
                     "\"error_code\":null,\"enabled\":null";
    out << ",\"homed\":" << f.homed << ",\"at_target\":" << f.at_target
        << ",\"current_threshold_a\":" << f.current_threshold_a
        << ",\"current_limit_reached\":" << f.current_limit_reached
        << ",\"holding_position\":" << f.holding_position
        << ",\"target_active\":" << f.target_active << ",\"target_position_m\":";
    if (f.target_active) out << f.target_position_m; else out << "null";
    out << "}" << std::endl;
}
bool parse(const std::string& line, gripper_demo::Command& c) {
    std::istringstream in(line); in.imbue(std::locale::classic());
    std::string action, extra;
    if (!(in >> action)) return false;
    if (action == "MOVE") { c.action = gripper_demo::Action::Move; if (!(in >> c.position_m >> c.speed_mps)) return false; }
    else if (action == "ZERO") c.action = gripper_demo::Action::Zero;
    else if (action == "ENABLE") c.action = gripper_demo::Action::Enable;
    else if (action == "DISABLE") c.action = gripper_demo::Action::Disable;
    else if (action == "CLEAR") c.action = gripper_demo::Action::Clear;
    else return false;
    return !(in >> extra);
}
void consume(const std::string& line, gripper_demo::Controller& controller, std::ostream& out) {
    if (line == "QUIT") { stopped = 1; return; }
    gripper_demo::Command c;
    auto result = gripper_demo::Result{false, "syntax: MOVE position_m speed_mps; ZERO/ENABLE/DISABLE/CLEAR"};
    if (parse(line, c)) result = controller.command(c);
    out << "{\"type\":\"ack\",\"accepted\":" << std::boolalpha << result.accepted
        << ",\"detail\":" << json_string(result.detail) << "}" << std::endl;
}
} // namespace
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: gripper_demo [--use-existing-zero] [--current-threshold-a A] [port [motor_id [baud]]]\n"
                     "Defaults: /dev/ttyUSB0 1 115200; current threshold 0.5 A\n"
                     "Input: CLEAR | ZERO | ENABLE | MOVE position_m speed_mps | DISABLE | QUIT\n"
                     "Output: feedback (10 Hz) and ack as JSON lines. ZERO actively opens the gripper.\n";
        return 0;
    }
    std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler);
    try {
        bool use_existing_zero = false;
        double current_threshold_a = 0.5;
        std::vector<std::string> args;
        for (int i = 1; i < argc; ++i) {
            const std::string arg(argv[i]);
            if (arg == "--use-existing-zero") use_existing_zero = true;
            else if (arg == "--current-threshold-a") {
                if (++i >= argc) throw std::invalid_argument("missing current threshold");
                std::size_t used = 0; const std::string value(argv[i]);
                current_threshold_a = std::stod(value, &used);
                if (used != value.size() || !std::isfinite(current_threshold_a) || current_threshold_a <= 0)
                    throw std::invalid_argument("current threshold must be finite and positive");
            }
            else args.push_back(arg);
        }
        if (args.size() > 3) throw std::invalid_argument("too many arguments; use --help");
        const auto integer = [](const char* value) {
            std::size_t used = 0; int n = std::stoi(value, &used);
            if (used != std::string(value).size()) throw std::invalid_argument("invalid integer option");
            return n;
        };
        // Vendor library may log to stdout. Keep machine messages on original stdout.
        std::cout.flush();
        const char* message_fd = std::getenv("GRIPPER_MESSAGE_FD");
        const int output_fd = dup(message_fd ? integer(message_fd) : STDOUT_FILENO);
        if (output_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0)
            throw std::runtime_error("cannot redirect driver logging");
        // Machine messages use a separate descriptor; all vendor logging stays on stderr.
        std::ofstream output("/proc/self/fd/" + std::to_string(output_fd));
        if (!output) { close(output_fd); throw std::runtime_error("cannot open message output"); }
        close(output_fd);
        gripper_demo::Controller controller(gripper_demo::make_hardware(
            !args.empty() ? args[0] : "/dev/ttyUSB0", args.size() > 1 ? integer(args[1].c_str()) : 1,
            args.size() > 2 ? integer(args[2].c_str()) : 115200), current_threshold_a);
        controller.init(use_existing_zero);
        std::string pending;
        auto next = std::chrono::steady_clock::now();
        while (!stopped) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next) { feedback(output, controller.sample()); next = now + std::chrono::milliseconds(100); }
            pollfd pfd{STDIN_FILENO, POLLIN, 0};
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(next - std::chrono::steady_clock::now()).count();
            const int rc = poll(&pfd, 1, static_cast<int>(std::max<std::int64_t>(0, ms)));
            if (rc < 0) { if (errno == EINTR) continue; throw std::runtime_error("stdin poll failed"); }
            if (rc > 0 && (pfd.revents & (POLLIN | POLLHUP))) {
                char buf[512]; const auto n = read(STDIN_FILENO, buf, sizeof(buf));
                if (n == 0) break;
                if (n < 0) { if (errno == EINTR) continue; throw std::runtime_error("stdin read failed"); }
                pending.append(buf, static_cast<std::size_t>(n));
                std::size_t end;
                while (!stopped && (end = pending.find('\n')) != std::string::npos) {
                    auto line = pending.substr(0, end); pending.erase(0, end + 1);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    consume(line, controller, output);
                }
                if (pending.size() > 4096) throw std::runtime_error("input line exceeds 4096 bytes");
            }
            if (rc > 0 && (pfd.revents & (POLLERR | POLLNVAL))) throw std::runtime_error("stdin unavailable");
        }
    } catch (const std::exception& e) { std::cerr << "gripper_demo: " << e.what() << '\n'; return 1; }
    return 0;
}
