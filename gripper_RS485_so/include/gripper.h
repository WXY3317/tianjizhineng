#pragma once

#include <memory>

#include "motor_driver.hpp"
#include "grip_pawl.h"

/**
 * @brief 夹爪控制器
 */
class GripperController {
   public:
    /**
     * @brief 通过 MotorDriverFactory 创建电机驱动并构造夹爪控制器。
     *
     * 仅完成电机驱动实例创建和夹爪控制器构造，调用方需自行调用 init() 和 enable()。
     *
     * create() 不补充或覆盖配置字段，调用方必须提供完整配置。
     *
     * @param config 完整的电机驱动配置。
     * @return 成功返回 GripperController 实例，失败返回 nullptr。
     */
    static std::shared_ptr<GripperController> create(
        const MotorDriverConfig &config);

    /**
     * @brief 构造夹爪控制器。
     *
     * @param motor 已创建好的电机驱动实例（由 MotorDriverFactory::create 生成）。
     */
    explicit GripperController(std::shared_ptr<MotorDriver> motor);

    ~GripperController() = default;

    // 封装

    double get_motor_pos();

    void motor_torque_cmd(float torque_percent);

    double get_motor_torque();

    // ──────────── 生命周期 ────────────

    /** @brief 初始化夹爪电机。 @return 0 表示成功，非 0 表示失败。 */
    uint8_t init();

    /** @brief 反初始化夹爪电机。 */
    void deinit();

    /** @brief 夹爪电机上电。 */
    void enable();

    /** @brief 夹爪电机下电。 */
    void disable();

    /** @brief 校准夹爪零位。 */
    bool set_zero();

    /**
     * @brief 自动张开夹爪到机械极限并标零。
     *
     * 接口会使能电机，以张开方向施加指定力矩；检测到夹爪连续停止后撤销
     * 力矩，并将当前位置设置为零位。张开力矩固定为额定力矩的 -15%，
     * 等待超时时会撤销力矩并返回失败。
     *
     * @return 标零成功返回 true，失败或超时返回 false。
     */
    bool initialize_zero();

    /** @brief 清除电机驱动器错误。 */
    void clear_error();

    /** @brief 刷新电机状态。  */
    bool refresh_state();

    // ──────────── 位置控制 ────────────

    /**
     * @brief 控制夹爪移动到指定距离。
     *
     * @param distance_m  目标夹爪距离，单位 m。
     * @param speed_mps   夹爪开合速度，单位 m/s，默认 0.0 由驱动器自行决定。
     * @return 下发到电机的角度 (deg)。
     */
    double set_gripper_distance(double distance_m, float speed_mps = 0.0f);

    /**
     * @brief 获取当前夹爪距离。
     * @return 当前夹爪距离，单位 m。
     */
    double get_gripper_distance();

    // ──────────── 速度控制 ────────────

    /**
     * @brief 控制夹爪以指定速度运动。
     *
     * @param speed 夹爪闭合速度，单位 m/s；正值闭合，负值张开。
     */
    void set_gripper_velocity(float speed);

    /**
     * @brief 获取当前夹爪开合速度。
     * @return 夹爪闭合速度，单位 m/s；正值闭合，负值张开。
     */
    float get_gripper_velocity();

    // ──────────── 力矩控制 ────────────

    /**
     * @brief 按夹爪输出力矩控制。
     * @param force_n 夹爪输出力矩，单位 N。
     */
    void set_gripper_torque(float force_n);

    /**
     * @brief 获取当前夹爪输出力矩。
     * @return 夹爪输出力矩，单位 N。
     */
    float get_gripper_torque();

    // ──────────── 夹持控制 ────────────

    /**
     * @brief 两阶段夹持物体。
     *
     * 先用较小力矩闭合夹爪；检测到夹爪停止（接触物体或到达闭合极限）后，
     * 在 0.5 s 内线性增加到最终夹持力矩并保持该力矩。调用方可通过
     * set_gripper_torque(0) 或其他控制命令解除夹持。
     *
     * @param approach_torque_percent 接近物体时的力矩，额定力矩百分比。
     * @param holding_torque_percent  最终夹持力矩，额定力矩百分比，
     *                                不得小于 approach_torque_percent。
     * @return 检测到停止并完成力矩线性增加后返回 true；失败或超时返回 false。
     */
    bool grip(float approach_torque_percent, float holding_torque_percent);

    // ──────────── 控制模式 ────────────

    /**
     * @brief 设置电机控制模式。
     *
     * @param mode 控制模式
     */
    void set_control_mode(uint8_t mode);

    /**
     * @brief 获取当前电机控制模式。
     * @return 控制模式值。
     */
    uint8_t get_control_mode() const;

    // ──────────── 状态查询 ────────────

    /** @brief 检查电机是否已初始化。 */
    bool inited() const;

    /** @brief 检查电机是否已上电。 */
    bool enabled() const;

    /** @brief 检查电机是否未上电。 */
    bool disabled() const;

    /** @brief 获取电机错误码。 */
    uint32_t get_error_id() const;

    /**
     * @brief 等待夹爪停止移动。
     *
     * 通过连续比较电机位置反馈判断夹爪是否到达机械极限或接触物体。
     *
     * @param timeout_ms             最长等待时间 (ms)。
     * @param sample_interval_ms     位置采样周期 (ms)。
     * @param position_threshold_deg 相邻采样允许的最大电机位置变化 (deg)。
     * @param stable_sample_count    判定停止所需的连续稳定采样次数。
     * @return 在超时前检测到停止返回 true，否则返回 false。
     */
    bool wait_until_stopped(int timeout_ms = 10000,
                            int sample_interval_ms = 100,
                            float position_threshold_deg = 0.3f,
                            int stable_sample_count = 3);

   private:
    void stop_torque_safely() noexcept;

    std::shared_ptr<MotorDriver> motor_;
    grip_pawl pawl_;

    double rated_current = 3.5; // 电机额定电流 (A)
};
