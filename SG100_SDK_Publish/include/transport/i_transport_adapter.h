/**
 * @file i_transport_adapter.h
 * @brief 通讯层适配器抽象接口
 *
 * 通讯层职责：
 *   - 封装传输细节 (CRC附加/验证、CANFD分片/重组等)
 *   - 对上层暴露统一的 Send/SetRxCallback 接口
 *   - 每次 rx_cb_ 调用传递一帧完整协议体 (不含 CRC)
 *
 * 协议体格式 (传入/传出 ITransportAdapter 的字节流):
 *   [0x48][0x4D][Len][Dst][Src][Cmd][Data...]   共 6 + Len 字节
 */

#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include "../sg100_types.h"

/**
 * @brief 接收回调类型
 *
 * 适配器完成帧重组和（对 RS485）CRC 校验后，以此回调向上层投递一帧完整协议体。
 * payload 指向适配器内部缓冲区，回调返回后不得继续持有该指针。
 *
 * @param payload  协议体字节流 [H0][H1][Len][Dst][Src][Cmd][Data...]
 * @param len      协议体总长 = 6 + Len
 */
using TransportRxCallback = std::function<void(const uint8_t* payload, uint16_t len)>;

/**
 * @brief 通讯适配器抽象接口
 */
class ITransportAdapter {
public:
    virtual ~ITransportAdapter() = default;

    // ---- 生命周期 ----
    virtual bool Open()  = 0;
    virtual void Close() = 0;
    virtual bool IsOpen() const = 0;

    /**
     * @brief 发送一帧协议体 (不含 CRC，不含分片)
     *
     * 适配器负责：
     *   RS485: 追加 CRC-32 后写入串口
     *   CANFD: 按 64 字节分片放入 TX 队列
     *
     * @param payload  协议体 [H0][H1][Len][Dst][Src][Cmd][Data...]
     * @param len      协议体长度
     * @return true=发送成功 / 已入队
     */
    virtual bool Send(const uint8_t* payload, uint16_t len) = 0;

    /**
     * @brief 注册接收回调
     *
     * 适配器完成帧重组后调用此回调，每次调用对应一帧完整协议体。
     */
    virtual void SetRxCallback(TransportRxCallback cb) = 0;

    /** @brief 适配器名称 (用于日志) */
    virtual std::string GetName() const = 0;

    /**
     * @brief 轮询接收 — 驱动接收缓冲区处理，完整帧通过 rx_cb_ 投递
     *
     * RS485 实现: 读取串口字节并解析完整帧
     * CANFD 默认实现: 空操作（由外部 InjectRxFrame 驱动）
     */
    virtual void PollReceive() {}

    /**
     * @brief 清空接收缓冲区 (模式切换前调用，避免接收到旧数据)
     *
     * RS485 实现: 清空串口缓冲和内部接收缓冲
     * CANFD 默认实现: 空操作
     */
    virtual void FlushRx() {}

    /**
     * @brief 同步发送并等待完整响应帧 (OTA / 模式切换专用)
     *
     * 发送 tx_payload，轮询直到收到有效帧或超时。
     * rx_body 输出为完整协议体 [H0][H1][Len][Dst][Src][Cmd][Data...] (不含 CRC)。
     *
     * RS485 实现: 发送后轮询串口直到收到有效响应或超时
     * CANFD 默认实现: 返回 false（不支持同步模式）
     *
     * @param tx_payload  发送协议体
     * @param tx_len      发送长度
     * @param rx_body     输出: 响应协议体
     * @param timeout_ms  超时毫秒数
     * @return true=成功收到有效响应
     */
    virtual bool SendAndAwait(const uint8_t* tx_payload, uint16_t tx_len,
                              std::vector<uint8_t>& rx_body,
                              uint32_t timeout_ms) {
        (void)tx_payload; (void)tx_len; (void)rx_body; (void)timeout_ms;
        return false;
    }

    /**
     * @brief [CANFD 专属] 向适配器注入一帧已收到的 CANFD 原始帧
     * RS485 默认实现: 返回 false
     */
    virtual bool InjectRxFrame(uint32_t can_id, const uint8_t* data, uint8_t data_len) {
        (void)can_id; (void)data; (void)data_len;
        return false;
    }

    /**
     * @brief [CANFD 专属] 从 TX 队列取出下一帧供 CAN 驱动发送
     * RS485 默认实现: 返回 false
     */
    virtual bool GetTxFrame(CANFD_Frame_t& out) {
        (void)out;
        return false;
    }

    /**
     * @brief [CANFD 专属] 检查 TX 队列是否有待发帧
     * RS485 默认实现: 返回 false
     */
    virtual bool HasTxFrame() const { return false; }

    // =========================================================================
    // 驱动模式 (主动/被动)
    // =========================================================================

    /**
     * @brief 设置驱动模式
     *
     * 须在 Open() 调用前设置。
     * 主动模式 (DRIVE_MODE_ACTIVE): SDK 内部调度线程驱动收发。
     * 被动模式 (DRIVE_MODE_PASSIVE): SDK 不启动后台线程，由外部通过
     *   PassiveInjectRx / PassiveGetTxBytes 驱动收发。
     */
    void SetDriveMode(SG100_DriveMode_e mode) { drive_mode_ = mode; }

    /** @brief 获取当前驱动模式 */
    SG100_DriveMode_e GetDriveMode() const { return drive_mode_; }

    // =========================================================================
    // 被动模式接口 (DRIVE_MODE_PASSIVE)
    // =========================================================================

    /**
     * @brief 注入外部收到的原始字节流
     *
     * 适配器内部完成帧同步与 CRC/完整性验证，提取完整帧后调用 rx_cb_。
     * 仅在被动模式下有意义；主动模式实现可返回 false。
     *
     * RS485 实现: 追加到流缓冲区，调用内部 DrainRxBuf() 解析帧（含 CRC 验证）
     * CANFD 实现: 按 64 字节分片委托给 InjectRxFrame() 完成流重组
     *
     * @param data  外部接收到的原始字节
     * @param len   字节数
     * @return true=成功; false=缓冲区溢出或未实现
     */
    virtual bool PassiveInjectRx(const uint8_t* data, uint16_t len) {
        (void)data; (void)len;
        return false;
    }

    /**
     * @brief 取出下一条待发送的字节序列（含帧封装与 CRC）
     *
     * 调用方取到字节后直接写入总线，无需额外处理。
     * 每次调用返回一条完整帧；队列为空时返回 false。
     * 仅在被动模式 RS485 下有效；CANFD 用户应使用 GetTxFrame()。
     *
     * @param out  输出: 待发字节序列（包含 CRC-32）
     * @return true=有数据; false=队列为空或未实现
     */
    virtual bool PassiveGetTxBytes(std::vector<uint8_t>& out) {
        (void)out;
        return false;
    }

    /** @brief 检查被动模式 TX 队列是否有待发字节帧 */
    virtual bool PassiveHasTxBytes() const { return false; }

protected:
    SG100_DriveMode_e drive_mode_ = DRIVE_MODE_ACTIVE;  ///< 默认主动模式
};
