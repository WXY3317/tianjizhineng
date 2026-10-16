/**
 * @file comm_scheduler.h
 * @brief 通信调度层 — 命令队列管理与后台通信线程
 *
 * 职责:
 *   - 维护实时调度队列 (cmd_queue_) 和透传队列 (passthrough_queue_)
 *   - 在后台线程 (SerialTask) 中按 10ms 周期驱动 RS485 帧发送与轮询
 *   - 实时模式: 空队列时自动从控制缓存生成默认 CMD_REALTIME_CTRL 帧
 *   - 透传模式: 按 FIFO 顺序发送队列中的命令，队列空时跳过
 *
 * 依赖注入 (通过构造函数 std::function 回调提供):
 *   - send_fn_          : 构建并发送一帧，由 Impl::SendMessage 实现
 *   - get_encoded_ctrl_ : 获取已编码的整手控制帧，由 Impl 提供
 *   - poll_receive_fn_  : 轮询适配器接收缓冲，由 Impl 提供
 *   - get_rx_count_fn_  : 获取已解析完整传输帧计数，用于等待本次响应
 *
 * 注意: 仅适用于 RS485 模式。CANFD 模式由外部主动驱动，不启动后台线程。
 */

#pragma once
#include "../sg100_types.h"
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <functional>
#include <chrono>

/**
 * @brief RS485 通信调度器
 */
class CommScheduler {
public:
    using SendFn        = std::function<bool(uint8_t dst, uint8_t cmd,
                                             const uint8_t* data, uint16_t len)>;
    using GetCtrlFn     = std::function<Hand_Control_t()>;
    using PollReceiveFn = std::function<void()>;
    using GetRxCountFn  = std::function<uint32_t()>;

    /**
     * @param device_name       设备名称 (用于日志)
     * @param work_mode         工作模式引用 (调度器按此决定走哪条队列)
     * @param send_fn           帧发送回调
     * @param get_encoded_ctrl  获取已编码整手控制帧回调 (实时模式空队列时使用)
     * @param poll_receive_fn   适配器接收轮询回调
     */
    CommScheduler(const std::string& device_name,
                  const std::atomic<SG100_WorkMode_e>& work_mode,
                  SendFn        send_fn,
                  GetCtrlFn     get_encoded_ctrl,
                  PollReceiveFn poll_receive_fn,
                  GetRxCountFn  get_rx_count_fn);

    ~CommScheduler();

    // =========================================================================
    // 线程生命周期
    // =========================================================================

    /** 启动后台 SerialTask 线程 (幂等, 已在运行时忽略) */
    void Start();

    /** 停止后台 SerialTask 线程并等待退出 */
    void Stop();

    /** 返回后台线程是否正在运行 */
    bool IsRunning() const { return is_running_; }

    // =========================================================================
    // 实时调度队列 (WORK_MODE_REALTIME)
    // =========================================================================

    void AddToQueue(const SG100_QueueCmd_t& cmd);
    SG100_QueueCmd_t GetFromQueue();

    // =========================================================================
    // 透传队列 (WORK_MODE_PASSTHROUGH)
    // =========================================================================

    void AddToPassthroughQueue(const SG100_QueueCmd_t& cmd);
    SG100_QueueCmd_t GetFromPassthroughQueue();
    bool IsPassthroughQueueEmpty();

    // =========================================================================
    // 队列冲刷 (模式切换时调用)
    // =========================================================================

    /** 将实时调度队列中全部指令发出后清空 */
    void FlushRealtimeQueue();

    /** 将透传队列中全部指令发出后清空 */
    void FlushPassthroughQueue();

private:
    // ---- 单次通信周期 ----
    void Communication(std::chrono::steady_clock::time_point cycle_deadline);
    void CommunicationPassthrough(std::chrono::steady_clock::time_point cycle_deadline);
    void PollReceiveUntil(std::chrono::steady_clock::time_point cycle_deadline);

    // ---- 后台线程入口 ----
    void SerialTask();

    // ---- 注入依赖 ----
    std::string                          device_name_;
    const std::atomic<SG100_WorkMode_e>& work_mode_;
    SendFn                               send_fn_;
    GetCtrlFn                            get_encoded_ctrl_fn_;
    PollReceiveFn                        poll_receive_fn_;
    GetRxCountFn                         get_rx_count_fn_;

    // ---- 实时调度队列 ----
    std::vector<SG100_QueueCmd_t> cmd_queue_;
    std::mutex                    queue_mutex_;

    // ---- 透传队列 ----
    std::vector<SG100_QueueCmd_t> passthrough_queue_;
    std::mutex                    passthrough_mutex_;

    // ---- 线程管理 ----
    std::thread      serial_thread_;
    std::atomic<bool> is_running_{false};
};
