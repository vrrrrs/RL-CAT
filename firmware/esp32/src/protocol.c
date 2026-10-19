/**
 * Frame protocol implementation (see rlca_protocol.h and
 * firmware/doc/协议规范_树莓派通信.md).
 *
 * No ESP-IDF dependencies: this file is host-testable.
 */
#include <string.h>

#include "rlca_protocol.h"

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE, table driven (512 B of flash/RAM)               */
/* ------------------------------------------------------------------ */
static const uint16_t s_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

uint16_t rlca_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len--) {
        crc = (uint16_t)((crc << 8) ^ s_crc16_table[((crc >> 8) ^ *data++) & 0xFF]);
    }
    return crc;
}

uint16_t rlca_crc16_ccitt(const uint8_t *data, uint16_t len)
{
    return rlca_crc16_update(0xFFFFu, data, len);
}

/* ------------------------------------------------------------------ */
/* framing                                                             */
/* ------------------------------------------------------------------ */
uint16_t rlca_frame_build(uint8_t *out, uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint8_t len)
{
    if (len > RLCA_FRAME_MAX_PAYLOAD) {
        return 0;
    }

    out[0] = RLCA_FRAME_HEAD0;
    out[1] = RLCA_FRAME_HEAD1;
    out[2] = len;
    out[3] = seq;
    out[4] = type;
    if (len && payload) {
        memcpy(&out[5], payload, len);
    }

    /* CRC covers LEN..PAYLOAD end (LEN + 3 bytes), little endian on the wire */
    uint16_t crc = rlca_crc16_ccitt(&out[2], (uint16_t)(len + 3u));
    out[5 + len]     = (uint8_t)(crc & 0xFF);
    out[5 + len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    return (uint16_t)(RLCA_FRAME_OVERHEAD + len);
}

void rlca_cmd_joint_payload(uint8_t payload[RLCA_LEN_CMD_JOINT],
                            uint8_t mode, uint8_t flags,
                            const float q_rad[RLCA_NUM_JOINTS],
                            int16_t kp_scale_centi, uint8_t led)
{
    memset(payload, 0, RLCA_LEN_CMD_JOINT);
    payload[RLCA_CJ_OFF_MODE]  = mode;
    payload[RLCA_CJ_OFF_FLAGS] = flags;

    for (uint8_t i = 0; i < RLCA_NUM_JOINTS; i++) {
        int16_t raw = q_rad ? rlca_rad_to_q(q_rad[i]) : 0;
        rlca_wr_i16(&payload[RLCA_CJ_OFF_Q + 2u * i], raw);
    }

    rlca_wr_i16(&payload[RLCA_CJ_OFF_KP_SCALE], kp_scale_centi);
    payload[RLCA_CJ_OFF_LED] = led;
}

void rlca_cmd_mode_payload(uint8_t payload[RLCA_LEN_CMD_MODE], uint8_t mode, uint8_t param)
{
    payload[0] = mode;
    payload[1] = param;
}

/* ------------------------------------------------------------------ */
/* receiver state machine                                              */
/* ------------------------------------------------------------------ */
void rlca_rx_init(rlca_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state = RLCA_RX_WAIT_HEAD0;
}

static void rx_reset(rlca_rx_t *rx)
{
    rx->state   = RLCA_RX_WAIT_HEAD0;
    rx->hdr_idx = 0;
    rx->pay_idx = 0;
    rx->crc_idx = 0;
}

void rlca_rx_poll(rlca_rx_t *rx, uint64_t now_us)
{
    if (rx->state != RLCA_RX_WAIT_HEAD0 &&
        (now_us - rx->last_byte_us) > RLCA_RX_FRAME_TIMEOUT_US) {
        rx->resync++;
        rx_reset(rx);
    }
}

uint32_t rlca_rx_feed(rlca_rx_t *rx, const uint8_t *data, uint16_t n,
                      uint64_t now_us, rlca_frame_cb_t cb, void *ctx)
{
    uint32_t delivered = 0;

    for (uint16_t i = 0; i < n; i++) {
        uint8_t b = data[i];
        rx->last_byte_us = now_us;

        switch (rx->state) {
        case RLCA_RX_WAIT_HEAD0:
            if (b == RLCA_FRAME_HEAD0) {
                rx->state = RLCA_RX_WAIT_HEAD1;
            }
            break;

        case RLCA_RX_WAIT_HEAD1:
            if (b == RLCA_FRAME_HEAD1) {
                rx->state   = RLCA_RX_HEADER;
                rx->hdr_idx = 0;
            } else if (b != RLCA_FRAME_HEAD0) {
                /* 0xAA kept us here, anything else is garbage */
                rx->state = RLCA_RX_WAIT_HEAD0;
            }
            break;

        case RLCA_RX_HEADER:
            rx->hdr[rx->hdr_idx++] = b;
            if (rx->hdr_idx == 1 && b > RLCA_FRAME_MAX_PAYLOAD) {
                /* spec §6 measure 2: reject oversized LEN, resync */
                rx->len_err++;
                rx_reset(rx);
            } else if (rx->hdr_idx == 3) {
                rx->pay_idx = 0;
                rx->state   = (rx->hdr[0] == 0) ? RLCA_RX_CRC : RLCA_RX_PAYLOAD;
                if (rx->state == RLCA_RX_CRC) {
                    rx->crc_idx = 0;
                }
            }
            break;

        case RLCA_RX_PAYLOAD:
            rx->payload[rx->pay_idx++] = b;
            if (rx->pay_idx >= rx->hdr[0]) {
                rx->crc_idx = 0;
                rx->state   = RLCA_RX_CRC;
            }
            break;

        case RLCA_RX_CRC: {
            rx->crc_bytes[rx->crc_idx++] = b;
            if (rx->crc_idx < 2) {
                break;
            }

            uint8_t len = rx->hdr[0];
            uint16_t crc = rlca_crc16_ccitt(rx->hdr, 3);
            if (len) {
                crc = rlca_crc16_update(crc, rx->payload, len);
            }
            uint16_t rx_crc = (uint16_t)(rx->crc_bytes[0] | ((uint16_t)rx->crc_bytes[1] << 8));

            if (crc == rx_crc) {
                rx->frames_ok++;
                if (cb) {
                    rlca_frame_t f;
                    f.type = rx->hdr[2];
                    f.seq  = rx->hdr[1];
                    f.len  = len;
                    if (len) {
                        memcpy(f.payload, rx->payload, len);
                    }
                    cb(&f, ctx);
                }
                delivered++;
            } else {
                rx->crc_err++;
            }
            rx_reset(rx);
            break;
        }

        default:
            rx_reset(rx);
            break;
        }
    }

    return delivered;
}

/* ------------------------------------------------------------------ */
/* telemetry accessors                                                 */
/* ------------------------------------------------------------------ */
bool rlca_tel_fast_parse(const rlca_frame_t *f, rlca_tel_fast_t *out)
{
    if (!f || !out || f->type != RLCA_TYPE_TEL_FAST || f->len != RLCA_LEN_TEL_FAST) {
        return false;
    }

    const uint8_t *p = f->payload;
    for (uint8_t i = 0; i < RLCA_NUM_JOINTS; i++) {
        out->q[i]   = rlca_rd_i16(&p[RLCA_TF_OFF_Q  + 2u * i]);
        out->qd[i]  = rlca_rd_i16(&p[RLCA_TF_OFF_QD + 2u * i]);
        out->tau[i] = rlca_rd_i16(&p[RLCA_TF_OFF_TAU + 2u * i]);
    }
    for (uint8_t i = 0; i < 4; i++) {
        out->quat[i] = rlca_rd_i16(&p[RLCA_TF_OFF_QUAT + 2u * i]);
    }
    for (uint8_t i = 0; i < 3; i++) {
        out->gyro[i] = rlca_rd_i16(&p[RLCA_TF_OFF_GYRO + 2u * i]);
    }
    out->gait_sin   = rlca_rd_i16(&p[RLCA_TF_OFF_GAIT_SIN]);
    out->gait_cos   = rlca_rd_i16(&p[RLCA_TF_OFF_GAIT_COS]);
    out->cycle_cnt  = rlca_rd_u16(&p[RLCA_TF_OFF_CYCLE]);
    out->voltage_mv = rlca_rd_u16(&p[RLCA_TF_OFF_VOLTAGE]);
    out->status     = p[RLCA_TF_OFF_STATUS];
    out->err_flags  = p[RLCA_TF_OFF_ERR];
    return true;
}

bool rlca_tel_status_parse(const rlca_frame_t *f, rlca_tel_status_t *out)
{
    if (!f || !out || f->type != RLCA_TYPE_TEL_STATUS || f->len != RLCA_LEN_TEL_STATUS) {
        return false;
    }

    const uint8_t *p = f->payload;
    out->uptime_ms = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    out->voltage_mv = rlca_rd_u16(&p[4]);
    out->mode       = p[36];
    out->status     = p[37];
    out->err_flags  = p[38];
    out->seq_lost   = p[39];
    return true;
}
