/**
  ******************************************************************************
  * @file    rlca_sts.h
  * @brief   Feetech STS 舵机协议的组帧与解帧（纯逻辑，可 PC 单测）。
  *
  *  ⚠️ 与官方库的关系：
  *     官方 `SC.c` 用全局指针 + malloc 实现 syncRead，在 20 KB RAM 的
  *     F103 上不可取（碎片 + 无 NULL 检查）。本模块【自实现】同一套线协议，
  *     但完全静态、无堆、可离线单测。官方库保留在 Middlewares/SCServo/
  *     作为协议参考与寄存器地址宏来源。
  *     依据：docs/33-firmware-plan.md §4 取舍 1。
  *
  *  STS 帧格式（伺服总线模块设计 §2.1）：
  *
  *    指令帧： 0xFF 0xFF ID LEN INSTR P... CHK
  *             LEN = 从 INSTR 到 CHK 之前的字节数 = 参数长度 + 2
  *             CHK = ~(ID + LEN + INSTR + 所有参数) & 0xFF
  *
  *    反馈帧： 0xFF 0xFF ID LEN ERR DATA... CHK
  *             ⚠️ 比指令帧多一个 ERR 字节 —— 最常见的解析错误来源！
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#ifndef RLCA_STS_H
#define RLCA_STS_H

#include <stdint.h>
#include <stdbool.h>

/*==============================================================================
 * 协议常量
 *============================================================================*/

#define STS_HEAD                    0xFFu
#define STS_BROADCAST_ID            0xFEu

#define STS_INST_PING               0x01u
#define STS_INST_READ               0x02u
#define STS_INST_WRITE              0x03u
#define STS_INST_REG_WRITE          0x04u
#define STS_INST_REG_ACTION         0x05u
#define STS_INST_SYNC_READ          0x82u
#define STS_INST_SYNC_WRITE         0x83u

/* 寄存器地址（来自官方 ST.h，已核实） */
#define STS_REG_MODEL_L             3u
#define STS_REG_MODEL_H             4u
#define STS_REG_ID                  5u
#define STS_REG_BAUD_RATE           6u
#define STS_REG_MIN_ANGLE_LIMIT_L   9u
#define STS_REG_MAX_ANGLE_LIMIT_L   11u
#define STS_REG_CW_DEAD             26u
#define STS_REG_CCW_DEAD            27u
#define STS_REG_OFS_L               31u      /* 中位偏置（EPROM，标定用） */
#define STS_REG_MODE                33u
#define STS_REG_TORQUE_ENABLE       40u
#define STS_REG_ACC                 41u
#define STS_REG_GOAL_POSITION_L     42u
#define STS_REG_GOAL_TIME_L         44u
#define STS_REG_GOAL_SPEED_L        46u
#define STS_REG_LOCK                55u
#define STS_REG_PRESENT_POSITION_L  56u
#define STS_REG_PRESENT_SPEED_L     58u
#define STS_REG_PRESENT_LOAD_L      60u
#define STS_REG_PRESENT_VOLTAGE     62u
#define STS_REG_PRESENT_TEMPERATURE 63u
#define STS_REG_MOVING              66u
#define STS_REG_PRESENT_CURRENT_L   69u

/* 建议一次 SyncRead 读回的反馈块：位置2+速度2+负载2+电压1+温度1+移动1 = 9 字节 */
#define STS_FB_START_ADDR           STS_REG_PRESENT_POSITION_L    /* = 56 */
#define STS_FB_DATA_LEN             9u

/** ⚠️ 反馈帧总长 = 15 字节，别数错。帧结构是：
 *      H0 H1 ID LEN ERR DATA[9] CHK
 *      = 2(头) + 1(ID) + 1(LEN) + 1(ERR) + 9(数据) + 1(CHK) = 15
 *    ⚠️ 比指令帧【多一个 ERR 字节】——这是最容易算错的地方
 *       （伺服总线模块设计 §2.4 专门警告过）。
 *    之前的 14 是漏算了 CHK，结果 parse 的长度检查必然失败、
 *    12 帧反馈被全部丢弃（P3 单测抓到）。 */
#define STS_FB_FRAME_LEN            (6u + STS_FB_DATA_LEN)        /* = 15 */

/* 错误码位（协议规范 §4.6 / 伺服总线模块设计 §2.5） */
#define STS_ERR_VOLTAGE             0x01u
#define STS_ERR_ANGLE               0x02u
#define STS_ERR_OVERHEAT            0x04u
#define STS_ERR_OVERCURRENT         0x08u
#define STS_ERR_ANGLE_LIMIT         0x10u
#define STS_ERR_OVERLOAD            0x20u

/* 缓冲区容量。
 * ⚠️ 必须容得下最坏情况：12 舵机的 SyncWrite 参数区
 *    = addr(1) + item_len(1) + 12×(ID + 2 字节位置) = 38 字节。
 * 协议 LEN 上限是 254，但我们的单帧最大需求远小于此；取 64 留足余量。 */
#define STS_MAX_PAYLOAD             64u
#define STS_MAX_FRAME               (6u + STS_MAX_PAYLOAD)

/** ⭐ PING 应答的数据区长度（= LEN - 2）。
 *
 *  ⚠️⚠️ 这个值我【改错过一次】，记录全过程以免再犯：
 *
 *     ① 最初代码写的是 exp_len = 0，但注释说不清为什么，看着可疑。
 *     ② 我拿一帧"上机抓包"（FF FF 01 04 03 28 00 CF，LEN=4）当标准，
 *        推算出 data_len = 2，于是把常量改成 2。
 *     ③ ⚠️ 但那帧是【USB-TTL 接在 UART2 上抢总线】时抓到的，
 *        来源本身就不可信 —— 当时操作者就质疑过它的真实性。
 *     ④ 后来用 `Y` 命令（不经过解析、直接 dump 原始字节）实测，
 *        真实 PING 应答是：
 *            FF FF 05 02 00 F8
 *            头(2) ID(1) LEN(1) ERR(1) CHK(1) = 6 字节
 *            LEN = 2  →  data_len = LEN - 2 = 0
 *            → 数据区长度确实是 0，**原来的 exp_len = 0 是对的**。
 *     ⑤ 结论：我基于一个来源可疑的帧，把本来正确的常量改错了，
 *        直接导致 `A` 命令 12 个 ID 全部误报 no response。
 *
 *  ⭐ 教训：**修改常量前必须先确证"参考数据"的来源可靠**。
 *     这次的 `Y` 命令（绕过解析层、直接看字节）才是可信的取证方式。
 *
 *  ⚠️ 判据是 LEN == 2（而不是 data_len 的算术结果），
 *     这样无论 LEN 怎么变，只要不是 2 就明确判定为失败。 */
#define STS_PING_FB_LEN             2u
#define STS_PING_FB_DATA_LEN        (STS_PING_FB_LEN - 2u)   /* = 0 */

/* 一条 SyncWrite 帧的最大参数区（12 舵机 × 3 + 2 = 38），编译期自检 */
#define STS_SYNC_WRITE_12_PARAMS    (2u + (12u * 3u))

/*==============================================================================
 * 组帧结果
 *============================================================================*/

typedef enum {
    STS_TX_OK = 0,
    STS_TX_ERR_NULL,        /* 空指针                    */
    STS_TX_ERR_CAPACITY,    /* 输出缓冲不足               */
    STS_TX_ERR_PARAM,       /* 参数非法（数量/长度/取值） */
    STS_TX_ERR_DUP_ID       /* SyncWrite 的 ID 有重复     */
} sts_tx_result_t;

/*==============================================================================
 * 反馈解析
 *============================================================================*/

/** 解析出的一帧舵机反馈。各字段是【原始值】，单位换算见 rlca_math.h */
typedef struct {
    uint8_t  id;
    uint8_t  err;              /* STS_ERR_* 位域                    */
    uint8_t  data[STS_MAX_PAYLOAD];
    uint8_t  data_len;         /* 实际收到的数据字节数               */
} sts_feedback_t;

/** 完整反馈帧解码的结果 */
typedef enum {
    STS_FB_OK = 0,             /* 收到一帧，且校验和/ID/ERR 均正常    */
    STS_FB_ERR_HEAD,           /* 帧头不是 FF FF                     */
    STS_FB_ERR_ID,             /* ID 与期望不符                      */
    STS_FB_ERR_CHK,            /* 校验和错误                         */
    STS_FB_ERR_LEN,            /* LEN 与期望不符                     */
    STS_FB_ERR_SERVO           /* 帧本身正常，但 ERR 字节非零         */
} sts_fb_result_t;

/** 反馈数据块打包（9 字节，小端）。
 *
 *  ⚠️ 这个函数放在头文件里，是为了让【假舵机模拟器】能与真实解析路径
 *     共用同一套字节布局 —— 否则测试和固件可能各写一份、各自有 bug，
 *     测试通过了真机却不通。 */
static inline void rlca_sts_pack_fb_data(uint8_t out[STS_FB_DATA_LEN],
                                         int16_t pos, int16_t speed, int16_t load,
                                         uint8_t voltage_dv, uint8_t temp_c,
                                         uint8_t moving)
{
    out[0] = (uint8_t)((uint16_t)pos & 0xFFu);
    out[1] = (uint8_t)(((uint16_t)pos >> 8) & 0xFFu);
    out[2] = (uint8_t)((uint16_t)speed & 0xFFu);
    out[3] = (uint8_t)(((uint16_t)speed >> 8) & 0xFFu);
    out[4] = (uint8_t)((uint16_t)load & 0xFFu);
    out[5] = (uint8_t)(((uint16_t)load >> 8) & 0xFFu);
    out[6] = voltage_dv;
    out[7] = temp_c;
    out[8] = moving;
}

/** 构造一帧完整的反馈帧（15 字节：FF FF ID LEN ERR DATA... CHK）。
 *  同样供模拟器与自检使用。
 *  @return 帧总长（= STS_FB_FRAME_LEN + 1 = 15） */
static inline uint8_t rlca_sts_build_feedback(uint8_t *out, uint8_t id, uint8_t err,
                                              const uint8_t data[STS_FB_DATA_LEN])
{
    out[0] = STS_HEAD;
    out[1] = STS_HEAD;
    out[2] = id;
    out[3] = (uint8_t)(STS_FB_DATA_LEN + 2u);      /* LEN = 数据 + ERR + CHK */

    uint32_t sum = (uint32_t)id + (uint32_t)out[3] + (uint32_t)err;
    out[4] = err;
    for (uint8_t i = 0u; i < STS_FB_DATA_LEN; ++i) {
        out[5u + i] = data[i];
        sum += data[i];
    }
    out[5u + STS_FB_DATA_LEN] = (uint8_t)(~sum & 0xFFu);
    return (uint8_t)(STS_FB_DATA_LEN + 6u);        /* = 15 */
}

/** 解析一帧完整反馈。
 *  @param buf     帧首地址（必须是 0xFF 0xFF 开头）
 *  @param n       可用字节数（≥ 5 + 期望数据长度）
 *  @param exp_id  期望的舵机 ID（STS_BROADCAST_ID 表示不校验）
 *  @param exp_len 期望的数据长度（data_len）
 *  @param out     解析结果
 *  @return 见 sts_fb_result_t；STS_FB_OK 与 STS_FB_ERR_SERVO 都会填充 out，
 *          其余失败情况 out 内容未定义。 */
sts_fb_result_t rlca_sts_parse_feedback(const uint8_t *buf, uint16_t n,
                                        uint8_t exp_id, uint8_t exp_len,
                                        sts_feedback_t *out);

/*==============================================================================
 * 组帧
 *============================================================================*/

sts_tx_result_t rlca_sts_build_ping(uint8_t id, uint8_t *buf, uint16_t cap, uint16_t *out_len);

sts_tx_result_t rlca_sts_build_read(uint8_t id, uint8_t addr, uint8_t rd_len,
                                    uint8_t *buf, uint16_t cap, uint16_t *out_len);

sts_tx_result_t rlca_sts_build_torque(uint8_t id, bool enable,
                                      uint8_t *buf, uint16_t cap, uint16_t *out_len);

/** SyncWrite 目标位置：一条广播帧同时写多个舵机（INST_SYNC_WRITE）。
 *  @param ids  舵机 ID 数组
 *  @param n    舵机数量（1..254）
 *  @param pos  目标位置数组（步）
 *  @param cap  buf 容量，需 ≥ 7 + 3*n
 *  @param out_len 写出的字节数
 *  @note 若需要同时设置 ACC/SPEED，另用 rlca_sts_build_sync_write()。 */
sts_tx_result_t rlca_sts_build_sync_write_pos(const uint8_t *ids, uint8_t n,
                                              const int16_t *pos,
                                              uint8_t *buf, uint16_t cap,
                                              uint16_t *out_len);

/** 通用 SyncWrite：从 addr 开始，每个舵机写 item_len 字节。
 *  @param data   按舵机顺序紧密排列的数据（n × item_len 字节）
 *  @param item_len 每个舵机的数据长度（1..4） */
sts_tx_result_t rlca_sts_build_sync_write(uint8_t addr, uint8_t item_len,
                                          const uint8_t *ids, uint8_t n,
                                          const uint8_t *data,
                                          uint8_t *buf, uint16_t cap,
                                          uint16_t *out_len);

/** SyncRead 请求：一条广播帧请求多个舵机回传同一段寄存器（INST_SYNC_READ）。
 *  @param cap 需 ≥ 8 + n */
sts_tx_result_t rlca_sts_build_sync_read(uint8_t addr, uint8_t rd_len,
                                         const uint8_t *ids, uint8_t n,
                                         uint8_t *buf, uint16_t cap,
                                         uint16_t *out_len);

#endif /* RLCA_STS_H */
