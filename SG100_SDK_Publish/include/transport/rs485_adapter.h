/**
 * @file rs485_adapter.h
 * @brief RS485 通讯适配器
 *
 * 职责:
 *   发送: 在协议体末尾追加 CRC-32/BZIP2 (4字节 LE)，写入串口
 *   接收: 读串口字节流，搜索帧头，验证 CRC，剥离 CRC 后调用 rx_cb_
 *
 * 帧格式 (串口):
 *   [H0][H1][Len][Dst][Src][Cmd][Data...][CRC32-LE]
 *   其中 [H0..Data...] 为协议体 (6 + Len 字节)，对上层透明。
 */

#pragma once
#include "i_transport_adapter.h"
#include "../serial/serial.h"
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
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

class SG100_SDK_API RS485Adapter : public ITransportAdapter {
public:
    using TxCallback = std::function<void(const uint8_t* payload, uint16_t len)>;

    struct Config {
        std::string   port;
        unsigned long baudrate    = 1000000;
        int           databyte    = 8;
        int           parity      = 0;   ///< 0=none 1=odd 2=even
        int           stopbyte    = 1;   ///< 1 or 2
        int           flowcontrol = 0;   ///< 0=none 1=software 2=hardware
        uint32_t      timeout_ms  = 100;
    };

    /**
     * @param cfg        串口配置
     * @param drive_mode 驱动模式:
     *   DRIVE_MODE_ACTIVE  (默认) — SDK 持有串口，Send() 直接写串口，PollReceive() 轮询读取
     *   DRIVE_MODE_PASSIVE          — SDK 不操作串口，Send() 缓存帧，外部通过
     *                                 PassiveInjectRx()/PassiveGetTxBytes() 驱动收发
     */
    explicit RS485Adapter(const Config& cfg,
                          SG100_DriveMode_e drive_mode = DRIVE_MODE_ACTIVE);
    ~RS485Adapter() override;

    // ---- ITransportAdapter ----
    bool Open()  override;
    void Close() override;
    bool IsOpen() const override;

    /**
     * @brief 发送协议体
     *
     * 主动模式: 追加 CRC-32，直接写串口
     * 被动模式: 追加 CRC-32，缓存到内部 TX 队列；外部通过 PassiveGetTxBytes() 取走
     */
    bool Send(const uint8_t* payload, uint16_t len) override;

    /** @brief 注册主动模式串口 write() 前的发送观测回调。 */
    void SetTxCallback(TxCallback cb) { tx_cb_ = std::move(cb); }

    void SetRxCallback(TransportRxCallback cb) override { rx_cb_ = std::move(cb); }
    std::string GetName() const override { return "RS485@" + cfg_.port; }

    /** 主动模式: 轮询串口并解析完整帧；被动模式: 空操作 */
    void PollReceive() override;

    /** 同步发送并等待响应帧 (OTA / 模式切换专用, 仅主动模式有效) */
    bool SendAndAwait(const uint8_t* tx_payload, uint16_t tx_len,
                      std::vector<uint8_t>& rx_body,
                      uint32_t timeout_ms) override;

    /** 清空串口接收缓存和内部 rx_buf_ (仅主动模式有效) */
    void FlushRx() override;

    // ---- 被动模式接口 ----

    /**
     * @brief 注入外部收到的原始字节流
     *
     * 内部完成帧同步（搜索 0x48 0x4D）、CRC-32 验证、帧剥离，
     * 验证通过的完整帧通过 rx_cb_ 投递（不含 CRC）。
     * 仅被动模式有效；主动模式返回 false。
     */
    bool PassiveInjectRx(const uint8_t* data, uint16_t len) override;

    /**
     * @brief 取出下一条待发字节序列（含帧封装与 CRC-32）
     *
     * 每次调用返回一帧；队列为空时返回 false。
     * 仅被动模式有效；主动模式返回 false。
     */
    bool PassiveGetTxBytes(std::vector<uint8_t>& out) override;

    /** @brief 检查被动模式 TX 队列是否有待发帧 */
    bool PassiveHasTxBytes() const override;

    const Config& GetConfig() const { return cfg_; }

private:
    Config              cfg_;
    serial::Serial      port_;
    TransportRxCallback rx_cb_;
    TxCallback          tx_cb_;
    bool                passive_open_ = false;  ///< 被动模式下的 "已打开" 标志

    // 接收流缓冲区 (主动/被动模式共用)
    static constexpr size_t RX_BUF_SIZE = 1024;
    uint8_t  rx_buf_[RX_BUF_SIZE];
    size_t   rx_len_ = 0;

    // 被动模式 TX 队列: 每个元素为一帧完整字节（含 CRC-32）
    std::vector<std::vector<uint8_t>> tx_passive_queue_;
    mutable std::mutex                tx_passive_mutex_;

    /** CRC-32/BZIP2 计算 */
    static uint32_t CalcCRC32(const uint8_t* data, uint16_t len);

    /**
     * @brief 从 rx_buf_ 中解析完整帧，逐帧调用 rx_cb_
     */
    void DrainRxBuf();
};
