/**
 * @file protocol_handler.h
 * @brief 协议解析层 — 将接收到的原始帧解码并分发到状态缓存
 *
 * 职责:
 *   - 解析 ParseResponse 中的全部 CMD，更新内部缓存
 *   - 持有整手状态、关节参数、掌板模式、模式切换同步等缓存状态
 *   - 通过回调向外部投递状态变化事件
 *
 * 依赖注入:
 *   - work_mode_      : 解析 CMD_RUNTIME_GET_MODE 时需要写入
 *   - decode_pos_fn_  : 线协议位置 → 用户角度的换算函数 (由 Impl 提供)
 */

#pragma once
#include "../sg100_types.h"
#include <string>
#include <mutex>
#include <atomic>
#include <functional>
#include <cstring>
#include <memory>

namespace spdlog {
class logger;
}

/**
 * @brief 协议解析与状态缓存管理器
 */
class ProtocolHandler {
public:
    using DecodePosFunc = std::function<float(uint8_t joint_idx, float wire_pos)>;

    /**
     * @param device_name     设备名称 (用于日志)
     * @param work_mode       工作模式原子变量引用 (解析 CMD_RUNTIME_GET_MODE 时写入)
     * @param decode_pos_fn   线协议位置 → 用户角度换算函数
     */
    ProtocolHandler(const std::string& device_name,
                    std::atomic<SG100_WorkMode_e>& work_mode,
                    DecodePosFunc decode_pos_fn);

    /** 初始化/重置所有缓存数据 */
    void InitStatusData();

    // =========================================================================
    // 帧解析主入口
    // =========================================================================

    /**
     * @brief 解析一条已经由适配器验证/重组的帧
     *
     * @param src_addr  源地址
     * @param cmd       命令字 (含或不含 0x80 响应标志)
     * @param data      载荷字节流 (可修改副本)
     * @param data_len  载荷长度
     */
    void HandleFrame(uint8_t src_addr, uint8_t cmd, uint8_t* data, uint8_t data_len);

    // =========================================================================
    // 回调注册
    // =========================================================================

    void RegisterHandStatusCallback(HandStatusCallback cb) { hand_status_cb_ = std::move(cb); }
    void RegisterErrorCallback(ErrorCallback cb)           { error_cb_ = std::move(cb); }

    // =========================================================================
    // 整手状态缓存读取
    // =========================================================================

    Hand_Status_t GetCachedHandStatus() const;
    float         GetCachedPosition(uint8_t joint_id) const;
    void          GetCachedPositions(float* positions_deg) const;
    float         GetCachedCurrent(uint8_t joint_id) const;
    float         GetCachedVelocity(uint8_t joint_id) const;
    uint32_t      GetStatusRecvCount() const;
    bool          GetJointMotorStatus(uint8_t joint_id, MotorUp_t& out) const;

    // =========================================================================
    // 关节参数缓存读取
    // =========================================================================

    bool GetJointParamDevice    (uint8_t joint_id, ParamDeviceGroup_t& out)          const;
    bool GetJointParamControl   (uint8_t joint_id, ParamControlGroup_t& out)         const;
    bool GetJointParamCalib     (uint8_t joint_id, ParamCalibrationGroup_t& out)     const;
    bool GetJointFocCalib       (uint8_t joint_id, FocZeroCalibrationResult_t& out)  const;
    bool GetJointRuntimeStatus  (uint8_t joint_id, RuntimeStatusFrame_t& out)        const;
    bool GetJointRawSnapshot    (uint8_t joint_id, RawSnapshotFrame_t& out)          const;
    bool GetJointSensorStatus   (uint8_t joint_id, SensorStatusFrame_t& out)        const;
    bool GetJointParamAdvControl(uint8_t joint_id, ParamAdvancedControlGroup_t& out) const;
    bool GetJointFwInfo         (uint8_t joint_id, FirmwareVersionInfo_t& out)        const;
    bool GetJointZeroResult     (uint8_t joint_id, JointZeroResult_t& out)            const;
    bool GetMt6826sFactoryCalStatus(uint8_t joint_id, Mt6826sFactoryCalStatus_t& out) const;

    // =========================================================================
    // 掌板运行模式缓存
    // =========================================================================

    /** 读取最近一次掌板运行模式查询结果; 若无有效数据返回 false */
    bool GetPalmRuntimeModeStatus(Palm_Runtime_Mode_Status_t& out) const;

    /** 使掌板运行模式缓存失效 (发出查询前调用) */
    void InvalidatePalmRuntimeMode();

    // =========================================================================
    // 模式切换同步 (用于 SwitchToPassthroughMode/RealtimeMode 中轮询等待响应)
    // =========================================================================

    /** 标记正在等待指定命令的模式切换响应 */
    void SetModeSwitchPending(uint8_t expected_cmd);
    void ClearModeSwitchPending();

    /** 当前是否仍在等待模式切换响应 */
    bool IsModeSwitchPending() const;

    /** 获取上一次模式切换是否成功 */
    bool GetModeSwitchResult() const;

    // =========================================================================
    // 状态抑制控制
    // =========================================================================

    void SetSuppressStatusQueryErrors(bool suppress) {
        suppress_status_query_errors_ = suppress;
    }

    void SetControlLogger(std::shared_ptr<spdlog::logger> logger);

private:
    void UpdateHandStatus(const uint8_t* data, uint8_t data_len);

    // ---- 注入依赖 ----
    std::string                       device_name_;
    std::atomic<SG100_WorkMode_e>&    work_mode_;
    DecodePosFunc                     decode_pos_fn_;

    // ---- 整手状态缓存 ----
    Hand_Status_t           cached_status_{};
    mutable std::mutex      status_mutex_;
    uint32_t                status_recv_count_ = 0;

    // ---- 关节参数缓存 ----
    ParamDeviceGroup_t           joint_param_device_[JOINT_COUNT]{};
    ParamControlGroup_t          joint_param_control_[JOINT_COUNT]{};
    ParamCalibrationGroup_t      joint_param_calib_[JOINT_COUNT]{};
    FocZeroCalibrationResult_t   joint_foc_calib_[JOINT_COUNT]{};
    RuntimeStatusFrame_t         joint_runtime_status_[JOINT_COUNT]{};
    RawSnapshotFrame_t           joint_raw_snapshot_[JOINT_COUNT]{};
    SensorStatusFrame_t          joint_sensor_status_[JOINT_COUNT]{};
    ParamAdvancedControlGroup_t  joint_param_adv_control_[JOINT_COUNT]{};
    FirmwareVersionInfo_t        joint_fw_info_[JOINT_COUNT]{};
    JointZeroResult_t            joint_zero_result_[JOINT_COUNT]{};
    Mt6826sFactoryCalStatus_t     joint_mt6826s_cal_[JOINT_COUNT]{};
    bool                          joint_mt6826s_cal_valid_[JOINT_COUNT]{};
    mutable std::mutex           joint_cache_mutex_;

    // ---- 掌板运行模式缓存 ----
    Palm_Runtime_Mode_Status_t   palm_runtime_mode_status_{};
    bool                         palm_runtime_mode_valid_ = false;
    mutable std::mutex           palm_runtime_mode_mutex_;

    // ---- 模式切换同步 ----
    mutable std::mutex mode_switch_mutex_;
    bool               mode_switch_pending_     = false;
    bool               mode_switch_result_      = false;
    uint8_t            pending_mode_switch_cmd_ = 0;

    // ---- 错误抑制 ----
    std::atomic<bool> suppress_status_query_errors_{false};

    std::shared_ptr<spdlog::logger> control_logger_;
    mutable std::mutex              control_logger_mutex_;

    // ---- 回调 ----
    HandStatusCallback hand_status_cb_;
    ErrorCallback      error_cb_;
};
