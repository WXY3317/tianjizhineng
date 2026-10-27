#pragma once
#include <stdint.h>
#include <string.h>

#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils.hpp"

class MotorDriver;

// CAN 与 RS485 共用的驱动创建参数。旧 create(...) 接口仍保留并转换成该结构。
struct MotorDriverConfig {
    uint16_t motor_id = 1;
    std::string protocol_type;
    std::string bus_type;
    std::string bus_interface;
    std::string motor_type;
    int motor_direction = 1;
    uint16_t master_id_offset = 0;
    double motor_zero_offset = 0.0;

    // 串行总线参数；CAN 驱动忽略该字段。
    int baud_rate = 38400;
};

using MotorCreator =
    std::function<std::shared_ptr<MotorDriver>(const MotorDriverConfig &)>;
class MotorDriverFactory {
   public:
    static MotorDriverFactory &instance();
    void register_motor(const std::string &type, MotorCreator creator);
    std::shared_ptr<MotorDriver> create(const MotorDriverConfig &config) const;
    std::shared_ptr<MotorDriver> create(uint16_t motor_id,                  // 电机id
                                        const std::string &protocol_type,  // 接口类型  kaiser、
                                        const std::string &bus_type,  // 总线接口  socketcan、peakcan
                                        const std::string &bus_interface, // 接口  can0、can1、can2
                                        const std::string &motor_type, // 电机型号
                                        int motor_dirction,            //电机方向
                                        uint16_t master_id_offset,     // 主id偏移
                                        double motor_zero_offset) const;  // 电机零位偏移

   private:
    std::unordered_map<std::string, MotorCreator> registry_;
};

class MotorDriver {
   public:
    enum MotorControlMode_e {
        NONE = 0,
        MIT = 1,
        POS = 2,
        SPD = 3,
        CUR = 4,
    };

    static constexpr double judgment_accuracy_threshold = 1e-2;
    static const int normal_sleep_time = 5;
    static const int setup_sleep_time = 500;

    MotorDriver(uint16_t motor_id, int motor_dirction, double motor_zero_offset) : motor_id_(motor_id), motor_control_mode_(0), motor_dirction_(motor_dirction), motor_zero_offset_(motor_zero_offset) {}
    virtual ~MotorDriver() = default;

    /**
     * @brief disable the motor .
     *
     * This function disable the motor.
     * Once disable, the motor will not respond to commands for movement.
     */
    virtual void disable() = 0;

    /**
     * @brief disable the motor the motor.
     *
     * This function disable.
     * After enable, the motor can respond to movement commands.
     */
    virtual void enable() = 0;

    /**
     * @brief Initializes the motor.
     *
     * This function initializes the motor for operation.
     * It performs necessary setup and configuration for motor control.
     *
     * @return True if motor initialization is successful; otherwise, false.
     */
    virtual uint8_t init_motor() = 0;

    /**
     * @brief Deinitializes the motor.
     *
     * This function deinitializes the motor.
     * It performs cleanup and releases resources associated with motor control.
     */
    virtual void deinit_motor() = 0;

    /**
     * @brief Sets the motor position to zero.
     *
     * This function sets the current motor position to zero.
     * It establishes a new reference point for position measurement.
     *
     * @return True if setting motor position to zero is successful; otherwise,
     * false.
     */
    virtual bool set_motor_zero() = 0;

    /**
     * @brief Commands the motor to move to a specified position at a specified
     * speed.
     *
     * This function is responsible for commanding the motor to move to a desired
     * position with a specified speed.
     *
     * @param pos The target position to move the motor to.
     * @param spd The speed at which the motor should move to the target position.
     * @param ignore_limit If true, ignores any position limits that may be set.
     */
    virtual void motor_pos_cmd(float pos, float spd) = 0;

    /**
     * @brief Commands the motor to rotate at a specified speed.
     *
     * This function commands the motor to rotate at the specified speed.
     *
     * @param spd The speed at which the motor should rotate.
     */
    virtual void motor_spd_cmd(float spd) = 0;

    /**
     * @brief Commands the motor to apply a specified torque percentage.
     *
     * This function commands the motor to apply the specified percentage of
     * rated torque/current.
     *
     * @param torque The command percentage, where 100 means 100%.
     */
    virtual void motor_torque_cmd(float torque) {
        (void)torque;
        throw std::logic_error("torque control is not supported by this motor driver");
    }

    /**
     * @brief Commands motor current in amperes when supported.
     *
     * @param current_ampere The target motor current in A.
     */
    virtual void motor_current_cmd(float current_ampere) {
        (void)current_ampere;
        throw std::logic_error("current control is not supported by this motor driver");
    }

    /**
     * @brief Commands the motor to operate in impedance mode with specific
     * parameters.
     *
     * This function sets the motor to operate in an impedance control mode, where
     * it applies force based on the provided parameters.
     *
     * @param f_p Proportional force value.
     * @param f_v Velocity-based force value.
     * @param f_kp Proportional stiffness coefficient.
     * @param f_kd Damping coefficient.
     * @param f_t Desired torque value.
     */
    virtual void motor_mit_cmd(float, float, float, float, float) {
        throw std::logic_error("MIT control is not supported by this motor driver");
    }

    /**
     * @brief Sets the control mode for the motor.
     *
     * This function specifies the control mode for the motor.
     *
     * @param motor_control_mode The control mode to be set for the motor.
     */
    virtual void set_motor_control_mode(uint8_t motor_control_mode) = 0;

    virtual void clear_motor_error() = 0;

    // 需要显式取得总线控制权的驱动可覆盖；CAN 默认无需额外操作。
    virtual void set_bus_control_enabled(bool) {}

    // CAN 驱动由接收回调持续更新状态，默认无需操作；RS485 驱动覆盖该函数主动轮询。
    virtual bool refresh_state() { return true; }

    /**
     * @brief Retrieves the ID of the motor.
     *
     * This function returns the unique identifier (ID) of the motor.
     *
     * @return The ID of the motor.
     */
    virtual uint8_t get_motor_id() { return motor_id_; }

    /**
     * @brief Retrieves the control mode of the motor.
     *
     * This function returns the current control mode of the motor.
     *
     * @return The control mode of the motor.
     */
    virtual uint8_t get_motor_control_mode() { return motor_control_mode_; }

    /**
     * @brief Retrieves the error ID associated with the motor.
     *
     * This function returns the error ID that indicates any error condition of
     * the motor.
     *
     * @return The error ID of the motor.
     */
    virtual uint32_t get_error_id() { return error_id_; }

    /**
     * @brief Retrieves the current position of the motor.
     *
     * This function returns the current position of the motor.
     *
     * @return The current position of the motor.
     */
    virtual float get_motor_pos() { return motor_pos_; }

    /**
     * @brief Retrieves the current speed of the motor.
     *
     * This function returns the current speed of the motor.
     *
     * @return The current speed of the motor.
     */
    virtual float get_motor_spd() { return motor_spd_; }

    /**
     * @brief Retrieves the current torque percentage of the motor.
     *
     * This function returns the current percentage of rated torque/current.
     *
     * @return The feedback percentage, where 100 means 100%.
     */
    virtual float get_motor_torque() { return motor_torque_; }

    /** @return Motor current feedback in A when supported. */
    virtual float get_motor_current() {
        throw std::logic_error("current feedback is not supported by this motor driver");
    }

    /**
     * @brief Retrieves the temperature of the motor.
     *
     * This function returns the current temperature of the motor.
     *
     * @return The temperature of the motor.
     */
    virtual float get_motor_temperature() { return motor_temperature_; }
    /**
     * @brief Retrieves the direction of the motor.
     *
     * This function returns the direction of the motor.
     *
     * @return The direction of the motor.
     */
    virtual int get_motor_dirction() { return motor_dirction_; }
    /**
     * @brief Checks if the motor is initialized.
     *
     * This function returns true if the motor is initialized, false otherwise.
     *
     * @return True if the motor is initialized, false otherwise.
     */

    virtual bool inited() { return inited_; }
    /**
     * @brief Checks if the motor is enabled.
     *
     * This function returns true if the motor is enabled, false otherwise.
     *
     * @return True if the motor is enabled, false otherwise.
     */
    virtual bool enabled() { return enabled_; }
    /**
     * @brief Checks if the motor is disabled.
     *
     * This function returns true if the motor is disabled, false otherwise.
     *
     * @return True if the motor is disabled, false otherwise.
     */
    virtual bool disabled() { return disabled_; }

    /**
     * @brief Performs a software reset when supported by the concrete driver.
     *
     * Drivers without this capability return an explicit error instead of
     * silently ignoring the request.
     */
    virtual void software_reset() {
        throw std::logic_error(
            "software_reset is not supported by this motor driver");
    }

    /**
     * @brief Immediately stops the motor when supported by the concrete driver.
     */
    virtual void emergency_stop() {
        throw std::logic_error(
            "emergency_stop is not supported by this motor driver");
    }

    /**
     * @brief Cancels an emergency stop when supported by the concrete driver.
     */
    virtual void cancel_emergency_stop() {
        throw std::logic_error(
            "cancel_emergency_stop is not supported by this motor driver");
    }

   protected:
    uint16_t motor_id_;
    uint8_t motor_control_mode_;  // 0:none 1:mit 2:pos 3:spd 4:cur

    std::atomic<uint32_t> error_id_{0};

    double motor_zero_offset_;
    int motor_dirction_;
    std::atomic<float> motor_pos_{0.f};
    std::atomic<float> motor_spd_{0.f};
    std::atomic<float> motor_torque_{0.f};
    std::atomic<float> motor_temperature_{0.f};
    std::atomic<bool> inited_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> disabled_{false};
};
