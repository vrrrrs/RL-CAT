/**
  ******************************************************************************
  * @file    servo_bus.h
  * @brief   舵机总线事务状态机（SyncWrite / SyncRead）。
  *
  *  ⭐ 这是整个固件最复杂的模块（伺服总线模块设计 §0）：
  *     ① 1 Mbps 下每字节 10 µs —— 错过一个字节就丢一帧
  *     ② 12 个舵机共享一条总线，要 SyncWrite/SyncRead 批量操作
  *     ③ 半双工总线的收发切换（turnaround）有时序要求
  *     ④ 【不能阻塞主循环】—— 100 Hz 节拍要保住
  *
  *  设计要点：
  *    · SyncWrite：一次性阻塞发出 44 字节 ≈ 440 µs（唯一允许阻塞的阶段）
  *    · SyncRead ：发出 20 字节请求后【不阻塞】，12 帧回复由 USART2 中断/DMA
  *                 收进环形缓冲，下一个控制周期开始时才解析
  *    · 整帧超时 3 ms 外挂（官方库的缺陷 1：它的超时是"字节间隔"而非整帧）
  *    · 零 malloc（官方库缺陷 2）；零动态分配
  *
  *  ⚠️ 端口抽象（rlca_servo_port_t）：
  *     把 BSP 的收发操作注入进来，这样【状态机可以在 PC 上用假舵机单测】，
  *     而不必依赖真实硬件。固件侧注入 bsp_uart 的实现，
  *     测试侧注入一个会"自动回标准 STS 反馈帧"的模拟器。
  ******************************************************************************
  */

#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>
#include <stdbool.h>

#include "rlcat_config.h"
#include "rlca_sts.h"
#include "rlca_math.h"

/*==============================================================================
 * 端口抽象（依赖注入）
 *============================================================================*/

typedef struct {
    /** 阻塞发送一段字节（超时在实现内部处理）。 */
    void (*write)(const uint8_t *data, uint16_t len);

    /** 当前可读字节数。 */
    uint16_t (*avail)(void);

    /** 读最多 n 字节，返回实际读出。 */
    uint16_t (*read)(uint8_t *dst, uint16_t n);

    /** 清空接收缓冲（每轮事务开始前调用，清掉残留）。 */
    void (*flush)(void);

    /** 微秒时间戳源（用于整帧超时与 turnaround）。 */
    uint32_t (*micros)(void);
} rlca_servo_port_t;

/*==============================================================================
 * 数据结构
 *============================================================================*/

/** 一个舵机的反馈（原始值，单位换算见 rlca_math.h） */
typedef struct {
    int16_t  pos_step;
    int16_t  speed_step;
    int16_t  load_raw;
    uint8_t  voltage_dv;      /* 0.1 V */
    uint8_t  temp_c;
    uint8_t  moving;
    uint8_t  err;
    uint8_t  valid;           /* 本帧是否解析成功 */
} servo_fb_t;

/** 总线统计（诊断，进 TEL_STATUS） */
typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;       /* 成功解析的反馈帧数            */
    uint32_t n_timeout;       /* 整帧超时次数（3 ms 硬上限）   */
    uint32_t n_idle_timeout;  /* 提前判"回复结束"的次数         */
    uint32_t n_crc_err;       /* 校验/格式错误次数             */
    uint32_t n_short;         /* 收到但不完整的帧数            */
    uint32_t n_ok_reads;      /* 完整读回 12 帧的轮次          */
    uint32_t n_failed_reads;  /* 未能读全 12 帧的轮次          */
    uint32_t err_accum;       /* 舵机 ERR 位累计（OR）         */
    uint32_t last_txn_us;     /* 上一轮事务耗时                */
    uint32_t max_txn_us;      /* 最大事务耗时                  */
} servo_bus_stats_t;

/** 总线状态机 */
typedef enum {
    SVB_IDLE = 0,
    SVB_WRITE,          /* 正在（阻塞）发 SyncWrite        */
    SVB_TURNAROUND,     /* 半双工方向切换等待               */
    SVB_READ_REQ,       /* （阻塞）发 SyncRead 请求          */
    SVB_READ_RSP,       /* 异步等 12 帧回复                 */
    SVB_PARSE,          /* 解析已收到的字节                 */
    SVB_DONE,
    SVB_TIMEOUT
} servo_bus_state_t;

/*==============================================================================
 * 对外接口
 *============================================================================*/

/** 绑定端口并初始化。
 *  @param port 端口实现（固件侧传 BSP 适配器；测试侧传模拟器）
 *  @param calib 标定表（提供 dir/offset/限位） */
void servo_bus_init(const rlca_servo_port_t *port, const rlca_calib_t *calib);

/** ⭐⭐⭐ 热更新标定表（标定完成 / 手动零点 / 恢复默认之后必须调用）。
 *
 *  ⚠️⚠️ 不调用会怎样：
 *     `servo_bus_init()` 里是 `s_calib = *calib;`（复制），所以标定完成后
 *     如果不刷新，`q_rad` 会一直用【上电时的旧表】算 ——
 *     表现为"R 里的 offset 是对的，但关节角是错的（±π）"，
 *     而且必须重启才生效。
 *
 *  ⚠️ 与 `servo_bus_init` 的区别：本函数【只】换表，
 *     不清统计、不清反馈、不动端口 —— 所以可以在运行期安全调用。 */
void servo_bus_set_calib(const rlca_calib_t *calib);

/** 总线状态机推进。主循环【高频】调用（每个周期至少调用若干次）。 */
void servo_bus_poll(void);

/** 发起一轮：SyncWrite 12 个目标 + SyncRead 12 个反馈。
 *  @param q_target 12 个关节目标角（rad，逻辑索引 A）
 *  @return true = 本轮已发起（不代表完成） */
bool servo_bus_start_cycle(const float *q_target);

/** ⭐⭐ 按【原始步数】发起一轮同步写（上电安全保持专用）。
 *
 *  ⚠️ 与 servo_bus_start_cycle() 的区别：【完全不经过角度域】——
 *     不查标定表、不做 rlca_q_clamp / rlca_q_to_step，
 *     只把步数夹到编码器物理范围 [0, 4095]。
 *
 *  用途：上电后"待在原地"。目标 = 当前实测步数 ⇒ 舵机无处可去。
 *        ⚠️ 必须走这个入口，因为未标定时【角度域的含义本身就是错的】：
 *           q_rad → rlca_q_clamp（按错限位夹）→ rlca_q_to_step（按错 offset 换算）
 *           ⇒ 得到一个固定的错步数 ⇒ 12 个舵机同向同速奔向它。
 *
 *  @param pos_step  12 个关节的目标步数（逻辑索引 A 顺序）
 *  @return false = 总线忙 / 端口未绑定 / 组帧失败 */
bool servo_bus_start_cycle_raw(const int16_t *pos_step);

/** 使用当前关节角重新发起一轮（不发新的 SyncWrite，只读）。 */
bool servo_bus_start_read_only(void);

/** 本阶段是否空闲（可以发起下一轮）。 */
bool servo_bus_is_idle(void);

/** 当前状态（诊断用）。 */
servo_bus_state_t servo_bus_state(void);

/** 12 个舵机的反馈（索引 = 逻辑索引 A）。 */
const servo_fb_t *servo_bus_feedback(void);

/** 从反馈算出的关节状态（q/qd/tau，已按 dir 修正符号）。
 *  ⚠️ 只有在 servo_bus_has_new_feedback() 为 true 时才应读取。 */
const rlca_joint_state_t *servo_bus_joint_states(void);

/** 本次读回是否产生了新数据（上轮 SyncRead 解析成功）。
 *  读取后自动清标志。 */
bool servo_bus_take_new_feedback(void);

/** 统计。 */
const servo_bus_stats_t *servo_bus_stats(void);

/** 12 个舵机温度的最大值（°C）。无有效数据时返回 0。
 *  ⚠️ 安全层用它判过温，所以必须是"有效帧里的最大值"，
 *     而不是"数组里的最大值"（无效项的 temp 可能是陈旧的）。 */
uint8_t servo_bus_max_temp(void);

/** 最近一轮是否读全了 12 帧反馈（位置反馈是否可信）。
 *  ACTIVE 准入用它：读不全就不知道关节实际在哪，不能跑策略。 */
bool servo_bus_feedback_fresh(void);

/** 力矩使能 / 卸载（单机指令，阻塞）。
 *  @return true = 帧已发出（不代表舵机收到） */
bool servo_bus_set_torque(uint8_t joint_idx, bool enable);

/** 全部舵机卸力。 */
void servo_bus_set_all_torque(bool enable);

/** ⭐ 暂停 / 恢复后台总线轮询（总线互斥）。
 *
 *  ⚠️⚠️ 为什么需要（实测抓到的竞态）：
 *     `bsp_shell_poll()` 与 `servo_bus_poll()` 在【同一个主循环】里跑，
 *     所以"后台同步读"与"shell 的单机指令"会交错：
 *        ① servo_bus_poll() 发出 SyncRead(1..12)
 *        ② 同一次主循环里 bsp_shell_poll() 执行 `A` → 又发出 PING
 *        ③ 总线上两家同时等回复，RX 里混着两种应答 → 双方都判失败
 *
 *  ⚠️ 用"暂停"而不是"abort"就够的原因：
 *     abort 只清当前状态，下一个 app_poll() 立刻又会发起新事务；
 *     必须同时阻止"发起"，才能真正把总线让给 shell。
 *
 *  ⚠️ servo_bus_ping() 内部已自动 pause/resume，
 *     标定流程等需要独占总线的地方可手动调用。
 *  ⚠️ 必须成对调用；暂停期间 10 Hz 控制环的舵机事务不会下发。 */
void servo_bus_pause(void);
void servo_bus_resume(void);

/** Ping 单个舵机（阻塞，带超时）。
 *  ⚠️ 内部自动 pause/resume，调用方无需自己处理互斥。 */
bool servo_bus_ping(uint8_t servo_id);

/** ⭐ 运行期调整总线时序（0 = 该项不变）。
 *
 *  ⚠️⚠️ 做成运行期可调是因为这几个值【只能靠实测确定】
 *     （取决于适配器、线长、舵机型号）。若做成编译期常量，
 *     每试一个值都要"改代码→重编译→重烧"，一轮 5 分钟。
 *
 *  @param turnaround_us    收发方向切换等待（1 Mbps 下 1 字节 = 10 µs，
 *                          原值 20 µs 只有 2 个字节时间，很可能太短）
 *  @param frame_timeout_us 整帧超时（原 3000）
 *  @param idle_gap_us      "回复结束"静默判据（原 200） */
void servo_bus_set_timing(uint16_t turnaround_us, uint16_t frame_timeout_us,
                          uint16_t idle_gap_us);

/** 读回当前总线时序。任一参数可传 NULL。 */
void servo_bus_get_timing(uint16_t *turnaround_us, uint16_t *frame_timeout_us,
                          uint16_t *idle_gap_us);

/** ⭐ 读诊断计数（用于算"应答成功率"，不依赖 n_ok_reads）。
 *  ⚠️ n_ok_reads 要求 12 帧全到，只接少数舵机时它永远是 0，
 *     看不出"到底通了没有"。这三个数才能算出真实成功率。 */
void servo_bus_get_diag(uint32_t *rounds, uint32_t *frames, uint32_t *bytes);

/** 诊断计数清零。 */
void servo_bus_reset_diag(void);

/** ⭐⭐ 取"最近一次被丢弃处的原始字节"（排查应答没被认出的问题）。
 *
 *  ⚠️ 为什么需要：修好 DMA 搬运后 `A`(PING) 报 id 5/9 都 ACK，
 *     但 `B`(SyncRead) 里只有 id 5 有数据 —— 说明有个应答到了却
 *     没被解析出来。而原代码丢弃时只 `n_short++`，看不到被拒的是什么。
 *
 *  @param dst        输出缓冲（可为 NULL，此时只取 out_*）
 *  @param dst_cap    dst 容量
 *  @param out_len    实际写入字节数
 *  @param out_off    被丢弃时在接收缓冲里的偏移
 *  @param out_total  被丢弃时接收缓冲的总长度 */
void servo_bus_get_trace(uint8_t *dst, uint16_t dst_cap, uint16_t *out_len,
                         uint16_t *out_off, uint16_t *out_total);

/** 最近见过的"不在期望 ID 列表里"的 ID（0 = 没遇到）。 */
uint8_t servo_bus_unknown_id(void);

/** 整帧到齐但校验/LEN 被拒收的累计次数。 */
uint32_t servo_bus_parse_fail_count(void);

/** 最近一次丢弃的原因：1=不在帧头 2=陌生ID 3=校验/LEN错（0=没丢过）。 */
uint8_t servo_bus_trace_reason(void);

/** 单关节直接控位（调试用，绕过 RL）。 */
bool servo_bus_write_one(uint8_t joint_idx, int16_t pos_step);

/** ⭐⭐ 按【原始步数】立刻写一个舵机的目标位置（阻塞发送，不走事务状态机）。
 *
 *  ⚠️ 与 servo_bus_write_one() 的区别：那个入口把参数当作【当前标定表下的
 *     步数目标】，物理位置会随标定表变化而漂移；本入口只把步数夹到编码器
 *     物理范围 [0,4095]，语义严格是"让这个舵机停在物理步数 pos_step"。
 *
 *  ⚠️ 会 bus_abort_now()（清掉在飞行中的事务）—— 所以调用方要保证
 *     不需要那一轮的反馈。用于"使能力矩前先把 12 个关节钉在当前位置"。 */
bool servo_bus_write_one_raw(uint8_t joint_idx, int16_t pos_step);

/** 清空接收缓冲与状态（总线异常后恢复用）。 */
void servo_bus_abort(void);

/** 是否发生了需要上报的总线错误（累计位）。
 *  位定义见 rlcat_config.h 的 RLCA_ERRF_*。
 *  ⚠️ BUS_TIMEOUT 位只有【连续 RLCA_BUS_FAIL_STREAK_ERR 轮】失败才置，
 *     单轮抖动不会置位（避免安全层幅度在 100%/75% 之间闪）。 */
uint8_t servo_bus_err_flags(void);

/** ⭐ 连续失败轮数（诊断用）。
 *  ⚠️ 与 `servo_bus_stats()->n_timeout` 的区别：
 *     n_timeout 是**累计**失败数（一个不漏）；
 *     本值是**当前连续**失败数（成功一轮就清零）。
 *     ⇒ n_timeout 涨而本值一直是 0~1 = 偶发抖动（正常）；
 *       本值 ≥ RLCA_BUS_FAIL_STREAK_ERR = 总线真的不通。 */
uint16_t servo_bus_fail_streak(void);

#endif /* SERVO_BUS_H */
