/**
  ******************************************************************************
  * @file    bsp_flash.c
  * @brief   标定参数 Flash 持久化实现。
  ******************************************************************************
  */

#include <string.h>

#include "bsp_flash.h"
#include "bsp_tick.h"
#include "rlca_crc16.h"

/*==============================================================================
 * 内部
 *============================================================================*/

static bool s_loaded = false;

/** 写入缓冲区：必须在 Flash 写入期间保持有效，且长度按 4 字节对齐补齐 0xFF */
#define FLASH_BUF_MAX  128u
static uint8_t s_wbuf[FLASH_BUF_MAX];

/** 简单的临界区：Flash 擦写期间 CPU 停摆，任何中断都会导致写失败 */
static inline uint32_t enter_critical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void exit_critical(uint32_t primask)
{
    if (primask == 0u) {
        __enable_irq();
    }
}

/** 读完整个页的首 N 字节（诊断用） */
void bsp_flash_dump_header(uint8_t out16[16])
{
    const uint8_t *p = (const uint8_t *)RLCA_CALIB_FLASH_ADDR;
    for (uint8_t i = 0u; i < 16u; ++i) {
        out16[i] = p[i];
    }
}

/*==============================================================================
 * 加载
 *============================================================================*/

bool bsp_flash_load_calib(rlca_calib_t *out)
{
    const uint8_t *page = (const uint8_t *)RLCA_CALIB_FLASH_ADDR;

    uint32_t magic;
    uint16_t version;
    uint16_t len;
    memcpy(&magic,   &page[BSP_FLASH_HDR_OFF + 0u], sizeof(magic));
    memcpy(&version, &page[BSP_FLASH_HDR_OFF + 4u], sizeof(version));
    memcpy(&len,     &page[BSP_FLASH_HDR_OFF + 6u], sizeof(len));

    if (magic != BSP_FLASH_HDR_MAGIC) {
        s_loaded = false;
        return false;
    }
    if (version != BSP_FLASH_HDR_VERSION) {
        s_loaded = false;
        return false;
    }
    if (len != (uint16_t)sizeof(rlca_calib_t)) {
        s_loaded = false;
        return false;
    }

    rlca_calib_t tmp;
    memcpy(&tmp, &page[BSP_FLASH_PAYLOAD_OFF], sizeof(tmp));

    /* 结构自洽性 + CRC 双重校验：CRC 能挡住"擦一半断电"这类损坏，
     * 结构校验能挡住"CRC 恰好对但字段非法"（例如 dir = 7） */
    if (!rlca_calib_is_valid(&tmp)) {
        s_loaded = false;
        return false;
    }

    uint16_t crc_stored;
    memcpy(&crc_stored, &page[BSP_FLASH_CRC_OFF], sizeof(crc_stored));

    /* ⚠️ 页内数据的 crc16 字段在 payload 里，但它不参与页级 CRC；
     *    页级 CRC 覆盖 0x00..0x5B（含 payload 里的 crc16 字段）。
     *    payload 自己的 crc16 由 rlca_calib_crc() 单独校验。 */
    const uint16_t crc_calc = rlca_crc16(page, BSP_FLASH_CRC_OFF);
    if (crc_calc != crc_stored) {
        s_loaded = false;
        return false;
    }

    if (out != NULL) {
        *out = tmp;
    }
    s_loaded = true;
    return true;
}

bool bsp_flash_calib_loaded(void)
{
    return s_loaded;
}

/*==============================================================================
 * 保存
 *============================================================================*/

/** 把页尾的"有效标志"也纳入：写入前把 payload 自带的 crc16 重算一遍 */
static bool flash_write_page_from(const uint8_t *src, uint16_t n)
{
    if (n > FLASH_BUF_MAX) {
        return false;
    }
    if ((n & 3u) != 0u) {
        return false;                    /* 长度必须 4 字节对齐 */
    }

    memcpy(s_wbuf, src, n);

    /* 页内其余字节保持擦除态（0xFF），只写前 n 字节 */
    HAL_StatusTypeDef st;
    const uint32_t addr = RLCA_CALIB_FLASH_ADDR;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef er = { 0 };
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = addr;
    er.NbPages     = 1u;

    uint32_t page_err = 0u;
    st = HAL_FLASHEx_Erase(&er, &page_err);
    if (st != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return false;
    }

    for (uint16_t off = 0u; off < n; off += 4u) {
        uint32_t word;
        memcpy(&word, &s_wbuf[off], sizeof(word));
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + off, word);
        if (st != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    (void)HAL_FLASH_Lock();
    return true;
}

bool bsp_flash_save_calib(const rlca_calib_t *in)
{
    if (in == NULL) {
        return false;
    }

    /* 构造整块镜像（96 B，4 字节对齐） */
    uint8_t page[FLASH_BUF_MAX];
    memset(page, 0xFF, sizeof(page));

    const uint32_t magic   = BSP_FLASH_HDR_MAGIC;
    const uint16_t version = BSP_FLASH_HDR_VERSION;
    const uint16_t len     = (uint16_t)sizeof(rlca_calib_t);

    /* payload 自带的 crc16 先重算，保证落盘数据自身自洽 */
    rlca_calib_t tmp = *in;
    tmp.crc16 = rlca_calib_crc(&tmp);

    memcpy(&page[BSP_FLASH_HDR_OFF + 0u], &magic,   sizeof(magic));
    memcpy(&page[BSP_FLASH_HDR_OFF + 4u], &version, sizeof(version));
    memcpy(&page[BSP_FLASH_HDR_OFF + 6u], &len,     sizeof(len));
    memcpy(&page[BSP_FLASH_PAYLOAD_OFF],  &tmp,     sizeof(tmp));

    const uint16_t crc = rlca_crc16(page, BSP_FLASH_CRC_OFF);
    memcpy(&page[BSP_FLASH_CRC_OFF], &crc, sizeof(crc));
    /* page[0x5E..0x5F] 已经是 0xFF */

    const uint16_t total = (uint16_t)(BSP_FLASH_CRC_OFF + 4u);   /* 96 */

    const uint32_t primask = enter_critical();
    const bool ok = flash_write_page_from(page, total);
    exit_critical(primask);

    if (!ok) {
        return false;
    }

    /* 回读校验：写完立刻读一次，确认真的落盘了。
     * ⚠️ 很多 Flash 问题是"写了但没生效"，不回读就发现不了。 */
    s_loaded = false;
    rlca_calib_t back;
    if (!bsp_flash_load_calib(&back)) {
        return false;
    }
    if (memcmp(&back, &tmp, sizeof(tmp)) != 0) {
        s_loaded = false;
        return false;
    }
    return true;
}

bool bsp_flash_erase_calib(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef er = { 0 };
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = RLCA_CALIB_FLASH_ADDR;
    er.NbPages     = 1u;

    uint32_t page_err = 0u;
    const uint32_t primask = enter_critical();
    const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &page_err);
    exit_critical(primask);

    (void)HAL_FLASH_Lock();

    s_loaded = false;
    return (st == HAL_OK);
}
