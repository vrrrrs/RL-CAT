/**
  ******************************************************************************
  * @file    rlca_sts.c
  * @brief   Feetech STS 协议组帧/解帧实现。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#include <string.h>

#include "rlca_sts.h"

/*==============================================================================
 * 内部辅助
 *============================================================================*/

/** 校验和：~(ID + LEN + INSTR + P...) & 0xFF */
static uint8_t sts_checksum(uint8_t id, uint8_t len, uint8_t instr,
                            const uint8_t *params, uint16_t nparams)
{
    uint32_t sum = (uint32_t)id + (uint32_t)len + (uint32_t)instr;
    for (uint16_t i = 0; i < nparams; ++i) {
        sum += params[i];
    }
    return (uint8_t)(~(sum & 0xFFu) & 0xFFu);
}

/** 写指令帧：0xFF 0xFF ID LEN INSTR [params...] CHK */
static sts_tx_result_t build_write(uint8_t id, uint8_t instr,
                                   const uint8_t *params, uint8_t nparams,
                                   uint8_t *buf, uint16_t cap, uint16_t *out_len)
{
    if ((buf == NULL) || (out_len == NULL)) {
        return STS_TX_ERR_NULL;
    }
    if ((params == NULL) && (nparams > 0u)) {
        return STS_TX_ERR_NULL;
    }
    if (nparams > STS_MAX_PAYLOAD) {
        return STS_TX_ERR_PARAM;
    }

    /* LEN = 参数数 + 2（INSTR + CHK）
     * ⚠️ 帧长 = LEN + 4：帧为 H0 H1 ID LEN [LEN 字节，含 CHK]
     *    ⇒ 2(头) + 1(ID) + 1(LEN) + LEN = LEN + 4
     *    例：Ping 的 LEN = 2 → 6 字节（0xFF 0xFF ID 02 01 CHK）✅
     *    ⚠️ 伺服总线模块设计 §6.1 把 SyncWrite 12 舵机写成 45 字节，
     *       那是按 LEN+5 算的，与它自己的 LEN 定义不自洽；实际为 44。 */
    const uint8_t len = (uint8_t)(nparams + 2u);
    const uint16_t total = (uint16_t)len + 4u;

    if (cap < total) {
        return STS_TX_ERR_CAPACITY;
    }

    buf[0] = STS_HEAD;
    buf[1] = STS_HEAD;
    buf[2] = id;
    buf[3] = len;
    buf[4] = instr;
    if (nparams > 0u) {
        memcpy(&buf[5], params, nparams);
    }
    buf[5u + nparams] = sts_checksum(id, len, instr, params, nparams);

    *out_len = total;
    return STS_TX_OK;
}

/*==============================================================================
 * 通用组帧
 *============================================================================*/

sts_tx_result_t rlca_sts_build_ping(uint8_t id, uint8_t *buf, uint16_t cap, uint16_t *out_len)
{
    return build_write(id, STS_INST_PING, NULL, 0u, buf, cap, out_len);
}

sts_tx_result_t rlca_sts_build_read(uint8_t id, uint8_t addr, uint8_t rd_len,
                                    uint8_t *buf, uint16_t cap, uint16_t *out_len)
{
    if (rd_len == 0u) {
        return STS_TX_ERR_PARAM;
    }
    const uint8_t params[2] = { addr, rd_len };
    return build_write(id, STS_INST_READ, params, 2u, buf, cap, out_len);
}

sts_tx_result_t rlca_sts_build_torque(uint8_t id, bool enable,
                                      uint8_t *buf, uint16_t cap, uint16_t *out_len)
{
    const uint8_t params[2] = { STS_REG_TORQUE_ENABLE, (uint8_t)(enable ? 1u : 0u) };
    return build_write(id, STS_INST_WRITE, params, 2u, buf, cap, out_len);
}

/*==============================================================================
 * SyncWrite / SyncRead
 *============================================================================*/

/** Sync 帧的公共组帧（ID 必须是广播 0xFE）。
 *  帧体 = ID(广播) LEN INSTR addr item_len [id data...] CHK
 *  ⚠️ 参数区【不】包含广播 ID 与指令码，但校验和必须把它们算进去。 */
static sts_tx_result_t build_sync(uint8_t instr, uint8_t addr, uint8_t item_len,
                                  const uint8_t *ids, uint8_t n,
                                  const uint8_t *data,
                                  uint8_t *buf, uint16_t cap, uint16_t *out_len)
{
    if ((buf == NULL) || (out_len == NULL) || (ids == NULL)) {
        return STS_TX_ERR_NULL;
    }
    if ((n == 0u) || (n > 254u)) {
        return STS_TX_ERR_PARAM;
    }
    if (item_len == 0u) {
        return STS_TX_ERR_PARAM;
    }
    if ((instr == STS_INST_SYNC_WRITE) && (data == NULL)) {
        return STS_TX_ERR_NULL;
    }
    if (instr == STS_INST_SYNC_WRITE) {
        /* SyncWrite：item_len 是"每个舵机要写的字节数"，
         * 受寄存器块大小限制（我们的用途是 2 字节位置） */
        if (item_len > 4u) {
            return STS_TX_ERR_PARAM;
        }
    }
    /* SyncRead：同一个字段是"每个舵机要读的字节数"，
     * ⚠️ 可以远大于 4（我们要读 9 字节的反馈块）；上限由参数区长度检查覆盖。
     *    （item_len 是 uint8_t，本来就不可能超过 255。） */

    /* ID 重复检查：重复会让舵机互相踩踏（总线冲突），属于配置错误 */
    for (uint8_t i = 0; i < n; ++i) {
        for (uint8_t j = (uint8_t)(i + 1u); j < n; ++j) {
            if (ids[i] == ids[j]) {
                return STS_TX_ERR_DUP_ID;
            }
        }
    }

    /* 参数区长度：addr + item_len + (写：n×(1+item_len)  / 读：n×1) */
    uint32_t nparams = 2u;
    if (instr == STS_INST_SYNC_WRITE) {
        nparams += (uint32_t)n * (1u + (uint32_t)item_len);
    } else {
        nparams += (uint32_t)n;
    }
    if (nparams > STS_MAX_PAYLOAD) {
        return STS_TX_ERR_PARAM;
    }

    /* LEN = 参数数 + 2（INSTR + CHK） */
    const uint8_t len = (uint8_t)(nparams + 2u);
    const uint16_t total = (uint16_t)len + 4u;
    if (cap < total) {
        return STS_TX_ERR_CAPACITY;
    }

    buf[0] = STS_HEAD;
    buf[1] = STS_HEAD;
    buf[2] = STS_BROADCAST_ID;
    buf[3] = len;
    buf[4] = instr;

    uint16_t p = 5u;
    buf[p++] = addr;
    buf[p++] = item_len;

    if (instr == STS_INST_SYNC_WRITE) {
        for (uint8_t i = 0; i < n; ++i) {
            buf[p++] = ids[i];
            memcpy(&buf[p], &data[(uint32_t)i * (uint32_t)item_len], item_len);
            p = (uint16_t)(p + item_len);
        }
    } else {
        for (uint8_t i = 0; i < n; ++i) {
            buf[p++] = ids[i];
        }
    }

    /* ⚠️ 校验和要把广播 ID、INSTR 以及从 buf[5] 开始的全部参数算进去 */
    buf[p] = sts_checksum(STS_BROADCAST_ID, len, instr, &buf[5], (uint16_t)nparams);

    *out_len = (uint16_t)(p + 1u);
    return STS_TX_OK;
}

sts_tx_result_t rlca_sts_build_sync_write(uint8_t addr, uint8_t item_len,
                                          const uint8_t *ids, uint8_t n,
                                          const uint8_t *data,
                                          uint8_t *buf, uint16_t cap,
                                          uint16_t *out_len)
{
    if ((item_len != 2u) && (item_len != 1u) && (item_len != 4u)) {
        return STS_TX_ERR_PARAM;
    }
    return build_sync(STS_INST_SYNC_WRITE, addr, item_len, ids, n, data, buf, cap, out_len);
}

sts_tx_result_t rlca_sts_build_sync_write_pos(const uint8_t *ids, uint8_t n,
                                              const int16_t *pos,
                                              uint8_t *buf, uint16_t cap,
                                              uint16_t *out_len)
{
    if (pos == NULL) {
        return STS_TX_ERR_NULL;
    }

    /* 把 int16 数组按小端铺成紧凑字节流（⚠️ 不能直接强转，见协议 §4.4 对齐警告） */
    uint8_t data[254 * 2];
    if (n > 254u) {
        return STS_TX_ERR_PARAM;
    }
    for (uint8_t i = 0; i < n; ++i) {
        /* ⚠️⚠️ 规范化到 [0, 4095]（2026-10-12）
         *
         *  Goal_Position 是【单圈 12 位绝对编码器】的目标，合法范围 0..4095。
         *  本函数原先写的是 `(uint16_t)pos[i]`：
         *    · 负数 ⇒ 变成 0xFFxx 形式（例如 −100 ⇒ 0xFF9C），
         *      而官方库 ST.c 对负数的约定是 `|p| | (1<<15)`（幅值 + 方向位），
         *      两者【不是一回事】。到底哪种对，需要 ST-3215 手册确认。
         *    · 超过 4095 ⇒ 越出编码器量程。
         *
         *  改成"按 4096 取模到 [0, 4095]"之后：
         *    · 结果一定是明确的合法位置，不依赖任何未确认的负数约定；
         *    · 与"4096 步一圈"的物理事实一致（step 与 step±4096 是同一位置）。
         *
         *  ⚠️ 对本项目实际使用的角域【行为不变】：
         *     一个关节在硬限位边界处的步数 = offset + dir×q_limit×SPR。
         *     标定后 offset 落在 [0,4095]；若某关节的边界步数越界，
         *     {offset, offset±4096} 的两种表示里必有一个落在 [0,4095] ——
         *     取模正是让该边界【可达】，而不是引入错误。
         *  ⚠️ 标定路径（servo_bus_write_one）与主控制 SyncWrite 都走本函数，
         *     已由 test_servo_bus（174 项）与 test_safety_calib 覆盖。 */
        int32_t p = (int32_t)pos[i];
        p %= 4096;
        if (p < 0) { p += 4096; }
        const uint16_t v = (uint16_t)p;
        data[(2u * i)]      = (uint8_t)(v & 0xFFu);
        data[(2u * i) + 1u] = (uint8_t)((v >> 8) & 0xFFu);
    }

    return build_sync(STS_INST_SYNC_WRITE, STS_REG_GOAL_POSITION_L, 2u,
                      ids, n, data, buf, cap, out_len);
}

sts_tx_result_t rlca_sts_build_sync_read(uint8_t addr, uint8_t rd_len,
                                         const uint8_t *ids, uint8_t n,
                                         uint8_t *buf, uint16_t cap,
                                         uint16_t *out_len)
{
    return build_sync(STS_INST_SYNC_READ, addr, rd_len, ids, n, NULL, buf, cap, out_len);
}

/*==============================================================================
 * 反馈解析
 *============================================================================*/

sts_fb_result_t rlca_sts_parse_feedback(const uint8_t *buf, uint16_t n,
                                        uint8_t exp_id, uint8_t exp_len,
                                        sts_feedback_t *out)
{
    if ((buf == NULL) || (out == NULL)) {
        return STS_FB_ERR_HEAD;
    }
    /* 先把输出清零：任何失败路径都不会留下上一帧的残留数据 */
    memset(out, 0, sizeof(*out));

    /* 最短反馈帧：头2 + ID + LEN + ERR + CHK = 6 字节 */
    if (n < 6u) {
        return STS_FB_ERR_HEAD;
    }
    if ((buf[0] != STS_HEAD) || (buf[1] != STS_HEAD)) {
        return STS_FB_ERR_HEAD;
    }

    const uint8_t id  = buf[2];
    const uint8_t len = buf[3];

    /* ⚠️ 反馈帧比指令帧多一个 ERR 字节：LEN = 数据长度 + 2 */
    if (len < 2u) {
        return STS_FB_ERR_LEN;
    }
    const uint8_t data_len = (uint8_t)(len - 2u);
    if (data_len > STS_MAX_PAYLOAD) {
        return STS_FB_ERR_LEN;
    }

    const uint16_t total = (uint16_t)data_len + 6u;   /* 头2+ID+LEN+ERR+data+CHK */
    if (n < total) {
        return STS_FB_ERR_HEAD;
    }

    if ((exp_id != STS_BROADCAST_ID) && (id != exp_id)) {
        return STS_FB_ERR_ID;
    }
    if (data_len != exp_len) {
        return STS_FB_ERR_LEN;
    }

    /* 校验和覆盖 ID, LEN, ERR, DATA... （不含两个头字节） */
    uint32_t sum = (uint32_t)id + (uint32_t)len + (uint32_t)buf[4];
    for (uint8_t i = 0; i < data_len; ++i) {
        sum += buf[5u + i];
    }
    const uint8_t chk = (uint8_t)(~(sum & 0xFFu) & 0xFFu);
    if (chk != buf[5u + data_len]) {
        return STS_FB_ERR_CHK;
    }

    out->id       = id;
    out->err      = buf[4];
    out->data_len = data_len;
    if (data_len > 0u) {
        memcpy(out->data, &buf[5], data_len);
    }

    if (out->err != 0u) {
        return STS_FB_ERR_SERVO;
    }
    return STS_FB_OK;
}
