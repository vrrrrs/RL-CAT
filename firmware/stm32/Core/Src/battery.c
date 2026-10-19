/**
  ******************************************************************************
  * @file    battery.c
  * @brief   电池电压监测实现（纯逻辑 + 端口驱动）。
  *
  *  ⚠️ 滤波用整数滑动平均而不是浮点 IIR：
  *     本工程已有多处浮点（关节换算、相位），但电压这里没必要 ——
  *     整数平均零漂移、可复现，而且能直接单测出精确的期望值。
  ******************************************************************************
  */

#include <string.h>

#include "battery.h"

/* 编译期自检：滤波长度必须是 2 的幂（用移位取平均） */
#if (RLCA_BATT_FILTER_LEN == 0) || ((RLCA_BATT_FILTER_LEN & (RLCA_BATT_FILTER_LEN - 1)) != 0)
#error "RLCA_BATT_FILTER_LEN must be a power of two"
#endif

#define FILT_SHIFT  (RLCA_BATT_FILTER_LEN == 1 ? 0 : \
                     RLCA_BATT_FILTER_LEN == 2 ? 1 : \
                     RLCA_BATT_FILTER_LEN == 4 ? 2 : \
                     RLCA_BATT_FILTER_LEN == 8 ? 3 : 4)

/*==============================================================================
 * 纯逻辑
 *============================================================================*/

uint16_t batt_adc_to_mv(uint16_t raw, float vref_mv, float div_num, float div_den)
{
    if ((div_den <= 0.0f) || (vref_mv <= 0.0f)) {
        return 0u;
    }
    if (raw > RLCA_ADC_FULL_SCALE) {
        raw = (uint16_t)RLCA_ADC_FULL_SCALE;
    }

    /* 电池mV = raw / 4095 × vref × (R1+R2)/R2
     * ⚠️ 乘除顺序：先乘后除，减少中间量被截断的误差。 */
    const float adc_mv = ((float)raw * vref_mv) / (float)RLCA_ADC_FULL_SCALE;
    const float batt_mv = (adc_mv * div_num) / div_den;

    if (batt_mv <= 0.0f) {
        return 0u;
    }
    if (batt_mv >= 65535.0f) {
        return 65535u;
    }
    return (uint16_t)(batt_mv + 0.5f);
}

uint16_t batt_adc_to_mv_default(uint16_t raw)
{
    return batt_adc_to_mv(raw, RLCA_ADC_VREF_MV,
                          RLCA_BATT_DIV_NUM, RLCA_BATT_DIV_DEN);
}

batt_level_t batt_classify(uint16_t mv)
{
    /* ⚠️ 用 ≤ 而不是 <：阈值本身就是"到达该电压即触发"，
     *    与 docs/30-firmware.md §6.4 的表格一致（≤10.5 / ≤9.9 / ≤9.3）。 */
    if (mv <= RLCA_BATT_FORCE_MV) {
        return BATT_LEVEL_FORCE;
    }
    if (mv <= RLCA_BATT_STOP_MV) {
        return BATT_LEVEL_STOP;
    }
    if (mv <= RLCA_BATT_WARN_MV) {
        return BATT_LEVEL_WARN;
    }
    return BATT_LEVEL_OK;
}

uint8_t batt_status_bits(uint16_t mv)
{
    uint8_t st = 0u;
    if (mv <= RLCA_BATT_WARN_MV)  { st |= RLCA_ST_BATT_WARN; }
    if (mv <= RLCA_BATT_STOP_MV)  { st |= RLCA_ST_BATT_STOP; }
    if (mv <= RLCA_BATT_FORCE_MV) { st |= RLCA_ST_BATT_FORCE; }
    return st;
}

/*==============================================================================
 * 有状态层
 *============================================================================*/

static const batt_port_t *s_port = NULL;
static batt_state_t s_st;

/* 标定参数（默认取 config 里的值，可用 battery_set_calibration 覆盖） */
static float s_vref_mv = RLCA_ADC_VREF_MV;
static float s_div_num = RLCA_BATT_DIV_NUM;
static float s_div_den = RLCA_BATT_DIV_DEN;

/* 滑动平均缓冲（整数，避免浮点漂移） */
static uint16_t s_filt[RLCA_BATT_FILTER_LEN];
static uint32_t s_filt_sum = 0u;
static uint8_t  s_filt_idx = 0u;
static uint8_t  s_filt_n   = 0u;      /* 已填入的样本数（未满时用真实个数平均） */

static uint32_t s_last_sample_ms = 0u;
static uint32_t s_last_poll_ms   = 0u;

/** ⚠️ 必须用显式标志，不能用 `s_last_poll_ms != 0` 当哨兵：
 *     真实机器上电时 millis() 就是 0，第一次采样会落在 t=0，
 *     用 0 当哨兵会让"采样节流"在头 100 ms 内反复触发，
 *     表现成"上电瞬间连采好几次"。单测正是在 t=0 抓到了这个。 */
static bool s_poll_started = false;

void battery_init(const batt_port_t *port)
{
    s_port = port;
    memset(&s_st, 0, sizeof(s_st));
    memset(s_filt, 0, sizeof(s_filt));
    s_filt_sum  = 0u;
    s_filt_idx  = 0u;
    s_filt_n    = 0u;
    s_last_sample_ms = 0u;
    s_last_poll_ms   = 0u;
    s_poll_started   = false;
}

void battery_set_calibration(float vref_mv, float div_num, float div_den)
{
    if (vref_mv > 0.0f) { s_vref_mv = vref_mv; }
    if (div_num > 0.0f) { s_div_num = div_num; }
    if (div_den > 0.0f) { s_div_den = div_den; }
}

void battery_poll(void)
{
    if (s_port == NULL) {
        return;
    }

    const uint32_t now = s_port->millis();

    /* ---- 采样节流：10 Hz 足够，没必要每个主循环都读 ----
     * ⚠️ 首次调用必须立刻采样，所以用 s_poll_started 而不是
     *    "s_last_poll_ms == 0" 当哨兵（见其声明处的说明）。 */
    if (s_poll_started &&
        ((now - s_last_poll_ms) < RLCA_BATT_SAMPLE_PERIOD_MS)) {
        /* 即使不采样，也要维护"失效"判定 */
        if (s_st.valid && ((now - s_last_sample_ms) > RLCA_BATT_STALE_MS)) {
            s_st.valid = 0u;
        }
        return;
    }
    s_last_poll_ms = now;
    s_poll_started = true;

    /* ---- 取一个采样 ---- */
    uint16_t raw = 0u;
    if (!s_port->read_raw(&raw)) {
        s_st.n_timeouts++;
        if (s_st.valid && ((now - s_last_sample_ms) > RLCA_BATT_STALE_MS)) {
            s_st.valid = 0u;
        }
        return;
    }

    s_st.adc_raw = raw;
    s_st.n_samples++;
    s_last_sample_ms = now;

    /* ---- 滑动平均 ---- */
    s_filt_sum -= s_filt[s_filt_idx];
    s_filt[s_filt_idx] = raw;
    s_filt_sum += raw;
    s_filt_idx = (uint8_t)((s_filt_idx + 1u) % RLCA_BATT_FILTER_LEN);
    if (s_filt_n < RLCA_BATT_FILTER_LEN) {
        s_filt_n++;
    }

    const uint16_t avg_raw = (uint16_t)(s_filt_sum / (uint32_t)s_filt_n);

    s_st.mv_raw = batt_adc_to_mv(raw, s_vref_mv, s_div_num, s_div_den);
    s_st.mv     = batt_adc_to_mv(avg_raw, s_vref_mv, s_div_num, s_div_den);
    s_st.level  = (uint8_t)batt_classify(s_st.mv);
    s_st.valid  = 1u;

    (void)FILT_SHIFT;   /* 保留：若将来改成移位平均，这里就是那个常量 */
}

const batt_state_t *battery_state(void)
{
    return &s_st;
}

batt_level_t battery_level(void)
{
    return (batt_level_t)s_st.level;
}

uint16_t battery_mv(void)
{
    return s_st.mv;
}

bool battery_is_valid(void)
{
    return s_st.valid != 0u;
}
