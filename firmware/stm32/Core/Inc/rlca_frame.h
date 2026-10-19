/**
  ******************************************************************************
  * @file    rlca_frame.h
  * @brief   树莓派 ↔ STM32 通信帧的编解码（纯逻辑，可 PC 单测）。
  *
  *  帧格式（协议规范 §2）：
  *
  *    ┌──────┬──────┬──────┬──────┬──────┬─────────────┬───────┐
  *    │ 0xAA │ 0x55 │ LEN  │ SEQ  │ TYPE │  PAYLOAD    │ CRC16 │
  *    └──────┴──────┴──────┴──────┴──────┴─────────────┴───────┘
  *      偏移0   偏移1   偏移2   偏移3   偏移4   偏移5      偏移5+LEN
  *
  *    · LEN  = PAYLOAD 长度（不含头、不含 CRC），最大 128
  *    · CRC16 = CRC-16/CCITT-FALSE，【小端序】，覆盖 LEN..PAYLOAD 末尾
  *              （即 LEN + 3 字节，不含两个头字节）
  *    · 帧总长 = 7 + LEN
  *
  *  ⚠️ 解码器状态必须与树莓派侧完全一致（协议规范 §6），三处容错：
  *     ① WAIT_HEAD1 收到 0xAA 时【留在本状态】（处理 AA AA 55）
  *     ② LEN > 128 直接丢弃并复位（防缓冲溢出）
  *     ③ 收半帧后带超时复位（防上位机崩溃导致永久卡住）
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#ifndef RLCA_FRAME_H
#define RLCA_FRAME_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"

/*==============================================================================
 * 数据结构
 *============================================================================*/

/** 解码出的一个完整帧 */
typedef struct {
    uint8_t type;                              /* RLCA_TYPE_*             */
    uint8_t seq;
    uint8_t len;                               /* payload 长度            */
    uint8_t payload[RLCA_FRAME_MAX_PAYLOAD];
} rlca_rx_frame_t;

/** 接收状态机 */
typedef enum {
    RLCA_RX_WAIT_HEAD0 = 0,
    RLCA_RX_WAIT_HEAD1,
    RLCA_RX_HEADER,        /* 收 LEN / SEQ / TYPE 三字节 */
    RLCA_RX_PAYLOAD,
    RLCA_RX_CRC_LO,
    RLCA_RX_CRC_HI
} rlca_rx_state_t;

typedef struct {
    rlca_rx_state_t state;
    uint8_t  len;
    uint8_t  seq;
    uint8_t  type;
    uint16_t idx;                              /* payload 已收字节数       */
    uint16_t crc_rx;                           /* 收到的 CRC（小端拼装中） */
    uint32_t last_byte_ms;                     /* 用于整帧超时             */

    /* 诊断计数（TEL_STATUS 上报） */
    uint32_t n_ok;
    uint32_t n_crc_err;
    uint32_t n_len_err;
    uint32_t n_timeout;
    uint32_t n_unknown_type;
    uint32_t n_byte_total;
} rlca_rx_decoder_t;

/** FEED 返回值 */
typedef enum {
    RLCA_FEED_NONE = 0,    /* 还没收齐一帧                        */
    RLCA_FEED_FRAME,       /* 收到一个 CRC 正确的完整帧            */
    RLCA_FEED_CRC_ERR,     /* 帧收齐但 CRC 错                     */
    RLCA_FEED_LEN_ERR      /* LEN 超限，状态机已复位               */
} rlca_feed_result_t;

/*==============================================================================
 * 小端读写辅助（⚠️ 协议是紧凑字节流，不是 C 结构体内存布局）
 *============================================================================*/

static inline void rlca_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline uint16_t rlca_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void rlca_put_i16(uint8_t *p, int16_t v)
{
    rlca_put_u16(p, (uint16_t)v);
}

static inline int16_t rlca_get_i16(const uint8_t *p)
{
    return (int16_t)rlca_get_u16(p);
}

static inline void rlca_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline uint32_t rlca_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/*==============================================================================
 * 接口
 *============================================================================*/

/** 复位解码器（上电 / 整帧超时）。 */
void rlca_rx_init(rlca_rx_decoder_t *d);

/** 喂入一个字节。
 *  @param now_ms 当前毫秒时间戳，用于整帧超时
 *  @param out    收齐且 CRC 正确时写入（可为 NULL）
 *  @return 见 rlca_feed_result_t */
rlca_feed_result_t rlca_rx_feed(rlca_rx_decoder_t *d,
                                uint8_t byte,
                                uint32_t now_ms,
                                rlca_rx_frame_t *out);

/** 整帧超时检查：主循环调用。超时则复位状态机并累加 n_timeout。
 *  @return true = 本次发生了超时复位 */
bool rlca_rx_check_timeout(rlca_rx_decoder_t *d, uint32_t now_ms);

/** 计算帧的 CRC：内部构造 LEN/SEQ/TYPE 三字节前缀再算，调用方不必拼接。 */
uint16_t rlca_frame_crc(const uint8_t *payload, uint8_t len, uint8_t seq, uint8_t type);

/** 组帧到缓冲区。
 *  @param buf     输出缓冲，容量须 ≥ 7 + len
 *  @param cap     buf 容量
 *  @param type    消息类型
 *  @param seq     序列号
 *  @param payload 负载（len == 0 时可传 NULL）
 *  @param len     负载长度
 *  @return 写入的字节数（= 7 + len）；参数非法或缓冲不足时返回 0 */
uint16_t rlca_frame_pack(uint8_t *buf, uint16_t cap,
                         uint8_t type, uint8_t seq,
                         const uint8_t *payload, uint8_t len);

/** 单帧解包（树莓派侧模拟器与自检用）：校验头、长度、CRC。
 *  @return true = 解析成功 */
bool rlca_frame_unpack(const uint8_t *buf, uint16_t n, rlca_rx_frame_t *out);

#endif /* RLCA_FRAME_H */
