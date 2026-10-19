/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   UART 抽象：中断/DMA 接收 + 环形缓冲 + 溢出计数。
  *
  *  设计要点（固件设计书 §5.1）：
  *    ① 环形缓冲 ≥ 256 B —— 舵机总线 12 帧回复约 240 字节，1 Mbps 下来得很密
  *    ② 【必须有】溢出计数器 —— 否则丢数据无感知，排查时全是猜
  *    ③ 提供 flush() —— 舵机总线每轮事务开始前要清掉残留字节
  *
  *  三路 UART 的接收机制不同（协议/固件设计书的约定）：
  *    USART1（树莓派 1 Mbps）：逐字节中断 → 环形缓冲，CPU 占用可接受，
  *                             因为收完 103 B 后主循环有充足时间处理
  *    USART2（舵机 1 Mbps）  ：DMA 循环 + IDLE 中断 → 环形缓冲。
  *                             12 帧回复是 1.8 ms 内约 180 字节，
  *                             逐字节中断有丢字节风险（伺服总线设计 §5）
  *    USART3（ESP32 115200）：逐字节中断 → 环形缓冲；发送走中断（非阻塞，esp_fwd.c）
  *
  *  ⚠️ 缓冲区的 ISR 侧只写 head、主循环侧只写 tail，
  *     两侧都不写对方的指针 —— 这是无锁 SPSC 环形缓冲的标准做法。
  ******************************************************************************
  */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include "rlcat_config.h"

/*==============================================================================
 * 类型
 *============================================================================*/

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  *buf;                 /* 环形缓冲（静态分配）              */
    uint16_t  size;                /* 恒为 2 的幂，可用 & (size-1) 取模 */

    volatile uint16_t head;        /* 【ISR 写】生产者                */
    volatile uint16_t tail;        /* 【主循环写】消费者              */

    volatile uint32_t rx_total;    /* 累计收到的字节数                 */
    volatile uint32_t rx_overflow; /* ⚠️ 因缓冲满而丢弃的字节数         */
    volatile uint32_t rx_errors;   /* HAL 报的接收错误（ORE/FE/NE/PE）  */

    /** ⚠️ 发送失败计数（超时/硬件错）。
     *  加这一项的起因：UART3 转发 TEL_FAST 时，若 ESP32 侧不读数据，
     *  HAL_UART_Transmit 会一直等到超时才返回 —— 而接收侧有 rx_errors
     *  可查、发送侧却什么都没有，会【静默丢帧】让人完全无感。
     *  有了它就能一眼看出"是不是 ESP32 没在收"。 */
    volatile uint32_t tx_errors;

    uint8_t  *dma_buf;             /* DMA 循环缓冲（仅 USART2 用）      */
    uint16_t  dma_size;
    volatile uint16_t dma_last;    /* 上次已搬运到的位置               */

    /** ⭐ DMA 接收因错误被重启的次数（仅 USART2 舵机总线用 DMA）。
     *  ⚠️ 加这一项的起因：实测发现一次 ORE 溢出会把 DMA 卡死，
     *     而原代码只记 rx_errors、【没有重启】—— 接收就静默死亡了。
     *     表现为"`A` 全 no response、`B` 里 pos/V/T 全是 0"，
     *     而 TX（`P` 命令）完全正常（TX 是阻塞发送，不走 DMA）。
     *     有这个计数就能区分"从没收到过"与"收到过、被一次错误打断后没恢复"。 */
    volatile uint32_t dma_restarts;

    /** ⭐ 收到接收错误、需要主循环重启 DMA 的标志（仅 USART2）。
     *
     *  ⚠️⚠️ 为什么要这个标志（实测踩到）：
     *     HAL 出错时的顺序是 —— ① UART_EndRxTransfer()【关掉 DMA 通道】
     *     ② HAL_UART_ErrorCallback()（回调里把 ErrorCode 清成 0）
     *     ③ 最后才轮到主循环。
     *     所以只在主循环里检查 huart->ErrorCode 是【永远看不到】的
     *     —— 标志已被回调清掉 → 检查恒为假 → **一次 ORE 永久杀掉接收**。
     *     硬件证据：dma_ccr 的 EN 位 = 0、cndtr 不递减、
     *     而 hal_ErrorCode = 0、hal_dma_state = 1（HAL 以为 READY，脱节）。
     *
     *  ⇒ 由【回调置标志】、主循环见标志后重启。
     *  （不在回调里直接重启：ISR 上下文调 HAL stop/start 有重入风险。） */
    volatile uint8_t rx_need_restart;

    uint8_t   rx_byte;             /* HAL 逐字节接收的落点              */
} bsp_uart_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/** ⭐⭐ 纯硬件层面的 USART2 接收字节计数。
 *
 *  ⚠️⚠️ 存在的理由（方法论修正）：排查"收不到数据"时，
 *     `rx_frames` / `bytes` 等指标【全部经过我自己的代码路径】，
 *     无法区分"总线上真没字节"与"我的搬运逻辑有 bug"。
 *     本计数直接读 DMA 的 CNDTR，只回答
 *     "USART2 接收线上到底有没有字节进来"。
 *
 *  判读：在涨 = 硬件在收（问题在我的软件）；不动 = 线上真没字节。 */
uint32_t bsp_uart_dma_hw_bytes(void);
void     bsp_uart_dma_hw_reset(void);

/** 初始化三路 UART 的接收机制并启动接收。必须在 MX_*_UART_Init() 之后调用。 */
void bsp_uart_init_all(void);

/** 取出可用字节数。 */
uint16_t bsp_uart_available(const bsp_uart_t *p);

/** 读取最多 n 字节，返回实际读出字节数。 */
uint16_t bsp_uart_read(bsp_uart_t *p, uint8_t *dst, uint16_t n);

/** 读一个字节；无数据返回 -1。 */
int16_t bsp_uart_getc(bsp_uart_t *p);

/** 阻塞发送（用默认超时）。返回 true = 全部发出。
 *  ⚠️ 默认超时 = 2 + n/64 ms，对 1 Mbps 上 103 B 的帧够用（约 1 ms 实际耗时）。
 *     失败会计入 p->tx_errors。 */
bool bsp_uart_write(bsp_uart_t *p, const uint8_t *d, uint16_t n);

/** 阻塞发送，显式指定超时。
 *  ⚠️ 给"不能拖累控制环"的场合用（UART3 转发取 1 ms）：
 *     超时就丢帧并累加 tx_errors，绝不长时间阻塞。 */
bool bsp_uart_write_to(bsp_uart_t *p, const uint8_t *d, uint16_t n, uint32_t timeout_ms);

/** 阻塞发送一个字节。 */
bool bsp_uart_putc(bsp_uart_t *p, uint8_t b);

/** 清空接收缓冲（保留计数器，不重置统计）。 */
void bsp_uart_flush(bsp_uart_t *p);

/** 复位接收错误标志并重新启动接收（用于从 ORE 等错误中恢复）。 */
void bsp_uart_restart_rx(bsp_uart_t *p);

/** 推进 DMA + IDLE 接收：主循环必须【高频】调用。
 *  只处理 USART2（DMA 接收）；其他两路是中断接收，无需调用。 */
void bsp_uart_poll_dma(void);

/* 三路端口（供上层 link_pi / servo_bus 使用） */
extern bsp_uart_t g_uart_pi;      /* USART1 */
extern bsp_uart_t g_uart_servo;   /* USART2 */
extern bsp_uart_t g_uart_esp;     /* USART3 */

#endif /* BSP_UART_H */
