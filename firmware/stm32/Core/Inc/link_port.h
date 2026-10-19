/**
  ******************************************************************************
  * @file    link_port.h
  * @brief   把 USART1 与时基适配成 link_pi 需要的端口。
  *
  *  ⚠️ 与 servo_port 同样的思路：link_pi 通过 rlca_link_port_t 做依赖注入，
  *     好处是【链路状态机能在 PC 上单测】—— 包括模型 A 的 4 ms 等待窗口、
  *     序列号丢帧统计、失联阈值，这些都只能靠确定性测试来证明。
  ******************************************************************************
  */

#ifndef LINK_PORT_H
#define LINK_PORT_H

#include <stdint.h>
#include <stdbool.h>

/** 链路端口：收发一段字节 + 时间源 */
typedef struct {
    /** 阻塞发送（超时在实现内部处理）。返回 true = 全部发出 */
    bool (*write)(const uint8_t *data, uint16_t len);

    /** ⭐ 镜像发送：把【同一份已组好的帧】再发一份给 ESP32（UART3）。
     *
     *  ⚠️ 为什么放在端口里而不是让 link_pi 直接调 esp_fwd_*：
     *     那会让 link_pi.c 直接依赖硬件层，PC 上就链不上了
     *     （host 测试不编译 esp_fwd.c，它拖着 HAL）。
     *     做成端口成员后，PC 侧注入一个假实现即可，
     *     而且还能【断言哪些帧该被镜像、哪些不该】。
     *
     *  镜像策略（docs/32-esp32-bridge.md §4.3）：
     *     TEL_FAST / HEARTBEAT → 镜像（ESP32 靠它做指令仲裁与存活判断）
     *     TEL_STATUS / TEL_ACK → 不镜像（ESP32 明确只读 status 字节，
     *                            不需要把完整遥测都转过去，省带宽）
     *
     *  可以为 NULL（表示不镜像）。 */
    bool (*write_mirror)(const uint8_t *frame, uint16_t len);

    /** 当前可读字节数 */
    uint16_t (*avail)(void);

    /** 读最多 n 字节，返回实际读出 */
    uint16_t (*read)(uint8_t *dst, uint16_t n);

    /** 清空接收缓冲 */
    void (*flush)(void);

    /** 微秒时间戳（模型 A 的等待窗口用） */
    uint32_t (*micros)(void);

    /** 毫秒时间戳（失联判定与整帧超时用） */
    uint32_t (*millis)(void);
} rlca_link_port_t;

/** 取得"真实硬件"端口（USART1 + DWT/HAL 时基）。 */
const rlca_link_port_t *link_port_hardware(void);

#endif /* LINK_PORT_H */
