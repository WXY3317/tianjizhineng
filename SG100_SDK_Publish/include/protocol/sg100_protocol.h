/**
 * @file sg100_protocol.h
 * @brief SG100 协议层 — 与传输方式无关的帧打包/解析
 *
 * 协议帧体格式 (Transport-agnostic body):
 *   [0x48][0x4D][Len][Dst][Src][Cmd][Data...]
 *    H0    H1   1B   1B   1B   1B   Len 字节
 *   共 6 + Len 字节，不含任何校验字节。
 *
 * CRC 由具体传输适配器负责（RS485 与 CANFD 适配器均追加/校验 CRC-32/BZIP2）。
 */

#pragma once
#include <cstdint>
#include <vector>
#include <functional>
#include <cstddef>

/**
 * @brief 一条解码后的协议消息
 *
 * data 指向来源缓冲区内部，回调返回后不得持有。
 */
struct SG100Message {
    uint8_t        dst;       ///< 目标地址
    uint8_t        src;       ///< 源地址
    uint8_t        cmd;       ///< 命令字
    const uint8_t* data;      ///< 载荷数据 (可为 nullptr, 当 data_len==0)
    uint8_t        data_len;  ///< 载荷长度
};

/** @brief 消息解码回调 */
using ProtocolMessageCallback = std::function<void(const SG100Message&)>;

/**
 * @brief 协议层工具类
 *
 * 所有方法均为静态，无状态，线程安全。
 */
class SG100Protocol {
public:
    static constexpr uint8_t  HEADER_0   = 0x48;  ///< 帧头第1字节 'H'
    static constexpr uint8_t  HEADER_1   = 0x4D;  ///< 帧头第2字节 'M'
    static constexpr uint16_t HEADER_SIZE = 6;    ///< 帧头段大小 [H0][H1][Len][Dst][Src][Cmd]

    /**
     * @brief 打包协议体
     *
     * 输出格式: [0x48][0x4D][Len][Dst][Src][Cmd][Data...]
     *
     * @param dst       目标地址
     * @param src       源地址
     * @param cmd       命令字
     * @param data      载荷数据 (可为 nullptr)
     * @param data_len  载荷长度
     * @return 协议体字节序列 (共 6 + data_len 字节)
     */
    static std::vector<uint8_t> Pack(uint8_t dst, uint8_t src, uint8_t cmd,
                                     const uint8_t* data, uint8_t data_len);

    /**
     * @brief 流式解包
     *
     * 在字节流中搜索完整帧，对每帧调用 cb。
     * 调用方维护流缓冲区；返回值表示已消费字节数，调用方应将这部分从缓冲区头部移除。
     *
     * @param stream  输入字节流
     * @param len     流长度
     * @param cb      每帧回调
     * @return 已消费字节数
     */
    static size_t Unpack(const uint8_t* stream, size_t len,
                         const ProtocolMessageCallback& cb);
};
