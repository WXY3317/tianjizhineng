/**
 * @file canfd_adapter.h
 * @brief CANFD 通讯适配器
 *
 * 职责:
 *   发送: 将协议体按 64 字节分片封装为 CANFD_Frame_t，放入 TX 队列
 *         由外部 CAN 驱动调用 GetTxFrame() 轮询取帧发送
 *   接收: 外部 CAN 驱动收到帧后调用 InjectRxFrame()
 *         内部流式拼接，按协议体 Len 字段重组完整帧，调用 rx_cb_
 *
 * 注意: 为兼容掌板串口协议解析，SDK 在 CANFD 载荷中仍追加 CRC-32/BZIP2。
 */

#pragma once
#include "i_transport_adapter.h"
#include "../sg100_types.h"
#include <vector>
#include <mutex>

#ifdef _WIN32
  #ifdef SG100_SDK_EXPORTS
    #define SG100_SDK_API __declspec(dllexport)
  #else
    #define SG100_SDK_API __declspec(dllimport)
  #endif
#else
  #define SG100_SDK_API
#endif

class SG100_SDK_API CANFDAdapter : public ITransportAdapter {
public:
    /** 默认驱动模式为 PASSIVE: CANFD 的物理层由外部 CAN 驱动负责 */
    CANFDAdapter();
    ~CANFDAdapter() override = default;

    // ---- ITransportAdapter ----
    bool Open()  override { is_open_ = true;  return true; }
    void Close() override { is_open_ = false; }
    bool IsOpen() const override { return is_open_; }

    /**
    * @brief 发送协议体 — 先追加 CRC-32/BZIP2，再分片为 ≤64 字节的 CANFD_Frame_t 并入 TX 队列
     *
      * CAN ID 由配置决定：默认 tx_start_id=0x200，且同一条消息分片按 +1 递增。
    * 接收端会在重组后校验并剥离 CRC，再向上层投递协议体。
     */
    bool Send(const uint8_t* payload, uint16_t len) override;

    /** @brief 设置 CANFD 传输配置 (线程安全) */
    void SetConfig(const CANFD_Config_t& cfg);

    /** @brief 获取当前 CANFD 传输配置 (线程安全) */
    CANFD_Config_t GetConfig() const;

    void SetRxCallback(TransportRxCallback cb) override { rx_cb_ = std::move(cb); }
    std::string GetName() const override { return "CANFD"; }

    /** CANFD 由外部 InjectRxFrame 驱动，PollReceive 为空操作 */
    void PollReceive() override {}
    // SendAndAwait: 使用基类默认实现 (return false)

    // ---- 被动模式通用接口 ----

    /**
     * @brief 注入外部收到的原始字节流
     *
     * 直接将 data 追加到内部流缓冲区，完成帧头搜索与长度校验后
     * 通过 rx_cb_ 投递完整协议帧（不受 CANFD 物理帧 64 字节限制约束）。
     */
    bool PassiveInjectRx(const uint8_t* data, uint16_t len) override;

    // ---- 外部 CAN 驱动调用的接口 ----

    /**
     * @brief 注入一帧已收到的 CANFD 原始帧
     *
     * 内部: 将 data[0..data_len) 追加到流缓冲区，
    *       搜索帧头 0x48 0x4D，按 Len 字段与尾部 CRC 判断帧完整性，
    *       CRC 校验通过后剥离 CRC，再调用 rx_cb_(body, body_len)。
     *
    * @param can_id    CAN 帧 ID (可按 cfg_ 的 rx_filter_min_id/rx_filter_max_id 过滤)
     * @param data      CANFD 帧载荷
     * @param data_len  载荷长度 (≤ 64)
     * @return false = 缓冲区溢出或参数非法
     */
    bool InjectRxFrame(uint32_t can_id, const uint8_t* data, uint8_t data_len) override;
    bool GetTxFrame(CANFD_Frame_t& out) override;
    bool HasTxFrame() const override;

private:
    bool                is_open_  = false;
    TransportRxCallback rx_cb_;

  CANFD_Config_t      cfg_{};
  mutable std::mutex  cfg_mutex_;

    // TX 队列
    std::vector<CANFD_Frame_t> tx_queue_;
    mutable std::mutex         tx_mutex_;

    // RX 流缓冲区
    uint8_t    rx_stream_[CANFD_RX_STREAM_BUF_SIZE];
    size_t     rx_stream_len_ = 0;
    std::mutex rx_mutex_;
};
