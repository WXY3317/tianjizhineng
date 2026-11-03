#include "controller.hpp"
#include "calibration.hpp"
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <gripper/msg/gripper_command.hpp>
#include <gripper/msg/gripper_state.hpp>
#include <gripper/msg/gripper_ack.hpp>
#include <cmath>
#include <limits>

// A single-threaded executor owns all SDK operations, including blocking ZERO.
class GripperNode : public rclcpp::Node {
 public:
  GripperNode() : Node("gripper") {
    const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const bool dual = declare_parameter<bool>("dual_gripper", false);
    const int baud = declare_parameter<int>("baud_rate", 115200);
    const double rate = declare_parameter<double>("feedback_rate_hz", 10.0);
    if (!std::isfinite(rate) || rate < 0.1 || rate > 100.0)
      throw std::invalid_argument("feedback_rate_hz must be 0.1..100");
    const auto default_calibration = ament_index_cpp::get_package_share_directory("gripper") + "/config/calibration.csv";
    if (dual) {
      const int left_id = declare_parameter<int>("left.motor_id", 1);
      const int right_id = declare_parameter<int>("right.motor_id", 2);
      if (left_id == right_id || left_id < 1 || left_id > 247 || right_id < 1 || right_id > 247)
        throw std::invalid_argument("left/right motor IDs must be distinct and within 1..247");
      const auto left_cal = declare_parameter<std::string>("left.calibration_path", default_calibration);
      const auto right_cal = declare_parameter<std::string>("right.calibration_path", default_calibration);
      // Validate both tables before opening the bus or initializing either device.
      gripper_demo::Calibration left_table(left_cal), right_table(right_cal);
      add_gripper("left", "left_gripper/", port, left_id, baud, left_cal);
      add_gripper("right", "right_gripper/", port, right_id, baud, right_cal);
    } else {
      const int id = declare_parameter<int>("motor_id", 1);
      const auto calibration = declare_parameter<std::string>("calibration_path", default_calibration);
      add_gripper("", "", port, id, baud, calibration);
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate), [this] { for (auto& side : sides_) publish_state(*side); });
    RCLCPP_INFO(get_logger(), "Ready on %s with %zu device(s); no automatic motion", port.c_str(), sides_.size());
  }
 private:
  struct Side {
    float speed_mps;
    std::unique_ptr<gripper_demo::Controller> controller;
    rclcpp::Publisher<gripper::msg::GripperState>::SharedPtr state;
    rclcpp::Publisher<gripper::msg::GripperAck>::SharedPtr ack;
    rclcpp::Subscription<gripper::msg::GripperCommand>::SharedPtr command;
  };
  void add_gripper(const std::string& name, const std::string& topic,
                   const std::string& port, int id, int baud, const std::string& calibration) {
    const auto prefix = name.empty() ? "" : name + ".";
    const bool zero = declare_parameter<bool>(prefix + "use_existing_zero", false);
    const double threshold = declare_parameter<double>(prefix + "current_threshold_a", 0.5);
    rcl_interfaces::msg::ParameterDescriptor speed_descriptor;
    speed_descriptor.read_only = true;
    speed_descriptor.description = "Gripper opening speed in m/s; set at launch and restart to change";
    const double configured_speed = declare_parameter<double>(prefix + "speed_mps", 0.005, speed_descriptor);
    if (!std::isfinite(configured_speed) || configured_speed <= 0 ||
        configured_speed > std::numeric_limits<float>::max())
      throw std::invalid_argument(prefix + "speed_mps must be finite, positive and representable as float");
    const float speed = static_cast<float>(configured_speed);
    if (speed <= 0) throw std::invalid_argument(prefix + "speed_mps is too small");
    auto side = std::make_unique<Side>();
    side->speed_mps = speed;
    side->controller = std::make_unique<gripper_demo::Controller>(
        gripper_demo::make_hardware(port, id, baud, calibration), threshold);
    side->controller->init(zero);
    side->state = create_publisher<gripper::msg::GripperState>(topic + "state", 10);
    side->ack = create_publisher<gripper::msg::GripperAck>(topic + "ack", 10);
    auto* target = side.get();
    side->command = create_subscription<gripper::msg::GripperCommand>(topic + "command", 10,
      [target](gripper::msg::GripperCommand::ConstSharedPtr msg) {
        gripper::msg::GripperAck ack;
        if (msg->action > gripper::msg::GripperCommand::CLEAR) {
          ack.accepted = false; ack.detail = "unknown action";
        } else {
          gripper_demo::Command command;
          command.action = static_cast<gripper_demo::Action>(msg->action);
          command.position_m = msg->position_m;
          command.speed_mps = target->speed_mps;
          const auto result = target->controller->command(command);
          ack.accepted = result.accepted; ack.detail = result.detail;
        }
        target->ack->publish(ack);
      });
    sides_.push_back(std::move(side));
  }
  void publish_state(Side& side) {
    const auto f = side.controller->sample();
    gripper::msg::GripperState out;
    out.stamp = now();
    out.valid = f.valid;
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    out.position_m = f.valid ? f.position_m : invalid;
    out.motor_position_rad = f.valid ? f.motor_position_rad : invalid;
    out.motor_current_a = f.valid ? f.motor_current_a : invalid;
    out.motor_torque_percent = f.valid ? f.motor_torque_percent : invalid;
    out.force_n = f.valid ? f.force_n : invalid;
    out.error_code = f.error_code;
    out.enabled = f.valid && f.enabled;
    out.homed = f.homed;
    out.at_target = f.at_target;
    out.current_threshold_a = f.current_threshold_a;
    out.current_limit_reached = f.current_limit_reached;
    out.holding_position = f.holding_position;
    out.target_active = f.target_active;
    out.target_position_m = f.target_active ? f.target_position_m : invalid;
    side.state->publish(out);
  }
  std::vector<std::unique_ptr<Side>> sides_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<GripperNode>();
    rclcpp::spin(node);
    node.reset(); // Controller destruction attempts disable and closes the serial port.
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("gripper"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
