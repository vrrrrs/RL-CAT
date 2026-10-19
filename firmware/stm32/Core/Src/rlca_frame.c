/**
  ******************************************************************************
  * @file    rlca_frame.c
  * @brief   通信帧编解码 + 接收状态机。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#include <string.h>

#include "rlca_frame.h"
#include "rlca_crc16.h"

/*==============================================================================
 * CRC
 *============================================================================*/

uint16_t rlca_frame_crc(const uint8_t *payload, uint8_t len, uint8_t seq, uint8_t type)
{
    const uint8_t prefix[3] = { len, seq, type };
    uint16_t crc = rlca_crc16_update(RLCA_CRC16_INIT, prefix, 3u);
    if ((payload != NULL) && (len > 0u)) {
        crc = rlca_crc16_update(crc, payload, len);
    }
    return crc;
}

/*==============================================================================
 * 组帧
 *============================================================================*/

uint16_t rlca_frame_pack(uint8_t *buf, uint16_t cap,
                         uint8_t type, uint8_t seq,
                         const uint8_t *payload, uint8_t len)
{
    if (buf == NULL) {
        return 0u;
    }
    if (len > RLCA_FRAME_MAX_PAYLOAD) {
        return 0u;
    }
    const uint16_t total = (uint16_t)RLCA_FRAME_OVERHEAD + (uint16_t)len;
    if (cap < total) {
        return 0u;
    }

    buf[0] = RLCA_FRAME_HEAD0;
    buf[1] = RLCA_FRAME_HEAD1;
    buf[2] = len;
    buf[3] = seq;
    buf[4] = type;

    if ((payload != NULL) && (len > 0u)) {
        memcpy(&buf[5], payload, len);
    }

    const uint16_t crc = rlca_frame_crc(payload, len, seq, type);
    /* ⚠️ CRC 小端序 */
    buf[5u + len]      = (uint8_t)(crc & 0xFFu);
    buf[5u + len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    return total;
}

/*==============================================================================
 * 接收状态机
 *============================================================================*/

void rlca_rx_init(rlca_rx_decoder_t *d)
{
    if (d == NULL) {
        return;
    }
    memset(d, 0, sizeof(*d));
    d->state = RLCA_RX_WAIT_HEAD0;
}

static void decoder_reset_frame(rlca_rx_decoder_t *d)
{
    d->state = RLCA_RX_WAIT_HEAD0;
    d->len   = 0u;
    d->seq   = 0u;
    d->type  = 0u;
    d->idx   = 0u;
    d->crc_rx = 0u;
}

rlca_feed_result_t rlca_rx_feed(rlca_rx_decoder_t *d,
                                uint8_t byte,
                                uint32_t now_ms,
                                rlca_rx_frame_t *out)
{
    if (d == NULL) {
        return RLCA_FEED_NONE;
    }

    d->n_byte_total++;
    d->last_byte_ms = now_ms;

    switch (d->state) {
    case RLCA_RX_WAIT_HEAD0:
        if (byte == RLCA_FRAME_HEAD0) {
            d->state = RLCA_RX_WAIT_HEAD1;
        }
        break;

    case RLCA_RX_WAIT_HEAD1:
        if (byte == RLCA_FRAME_HEAD1) {
            d->state = RLCA_RX_HEADER;
            d->idx = 0u;
        } else if (byte == RLCA_FRAME_HEAD0) {
            /* ⚠️ 容错①：AA AA 55 —— 留在本状态，把当前这个 0xAA 当作新的头 */
            /* state 已经是 WAIT_HEAD1，无需改动 */
        } else {
            d->state = RLCA_RX_WAIT_HEAD0;
        }
        break;

    case RLCA_RX_HEADER:
        if (d->idx == 0u) {
            /* LEN */
            if (byte > RLCA_FRAME_MAX_PAYLOAD) {
                /* ⚠️ 容错②：LEN 超限直接丢弃并复位 */
                d->n_len_err++;
                decoder_reset_frame(d);
                return RLCA_FEED_LEN_ERR;
            }
            d->len = byte;
        } else if (d->idx == 1u) {
            d->seq = byte;
        } else {
            d->type = byte;
        }
        d->idx++;
        if (d->idx >= 3u) {
            d->idx = 0u;
            d->state = (d->len > 0u) ? RLCA_RX_PAYLOAD : RLCA_RX_CRC_LO;
        }
        break;

    case RLCA_RX_PAYLOAD:
        if (out != NULL) {
            out->payload[d->idx] = byte;
        }
        d->idx++;
        if (d->idx >= d->len) {
            d->idx = 0u;
            d->state = RLCA_RX_CRC_LO;
        }
        break;

    case RLCA_RX_CRC_LO:
        d->crc_rx = (uint16_t)byte;
        d->state = RLCA_RX_CRC_HI;
        break;

    case RLCA_RX_CRC_HI:
    {
        d->crc_rx |= (uint16_t)((uint16_t)byte << 8);

        const uint8_t *pl = (out != NULL) ? out->payload : NULL;
        const uint16_t crc_calc = rlca_frame_crc(pl, d->len, d->seq, d->type);

        if (crc_calc == d->crc_rx) {
            if (out != NULL) {
                out->type = d->type;
                out->seq  = d->seq;
                out->len  = d->len;
            }
            d->n_ok++;
            decoder_reset_frame(d);
            return RLCA_FEED_FRAME;
        }

        d->n_crc_err++;
        decoder_reset_frame(d);
        return RLCA_FEED_CRC_ERR;
    }

    default:
        decoder_reset_frame(d);
        break;
    }

    return RLCA_FEED_NONE;
}

bool rlca_rx_check_timeout(rlca_rx_decoder_t *d, uint32_t now_ms)
{
    if (d == NULL) {
        return false;
    }
    if (d->state == RLCA_RX_WAIT_HEAD0) {
        return false;                     /* 空闲，无超时概念 */
    }
    if ((uint32_t)(now_ms - d->last_byte_ms) > RLCA_FRAME_RX_TIMEOUT_MS) {
        d->n_timeout++;
        decoder_reset_frame(d);
        return true;
    }
    return false;
}

/*==============================================================================
 * 单帧解包（自检 / 树莓派模拟器用）
 *============================================================================*/

bool rlca_frame_unpack(const uint8_t *buf, uint16_t n, rlca_rx_frame_t *out)
{
    if ((buf == NULL) || (n < RLCA_FRAME_OVERHEAD)) {
        return false;
    }
    if ((buf[0] != RLCA_FRAME_HEAD0) || (buf[1] != RLCA_FRAME_HEAD1)) {
        return false;
    }

    const uint8_t len = buf[2];
    if (len > RLCA_FRAME_MAX_PAYLOAD) {
        return false;
    }
    const uint16_t total = (uint16_t)RLCA_FRAME_OVERHEAD + (uint16_t)len;
    if (n < total) {
        return false;
    }

    const uint8_t *pl = (len > 0u) ? &buf[5] : NULL;
    const uint16_t crc_calc = rlca_frame_crc(pl, len, buf[3], buf[4]);
    const uint16_t crc_rx   = rlca_get_u16(&buf[5u + len]);
    if (crc_calc != crc_rx) {
        return false;
    }

    if (out != NULL) {
        out->type = buf[4];
        out->seq  = buf[3];
        out->len  = len;
        if (len > 0u) {
            memcpy(out->payload, pl, len);
        }
    }
    return true;
}
