/**
  ******************************************************************************
  * @file    bsp_flash.h
  * @brief   标定参数的 Flash 持久化（最后一页 0x0800FC00）。
  *
  *  ⚠️ F103C8 的 Flash：64 KB、页大小 1 KB、共 64 页。
  *     标定数据放最后一页，固件只用到 0x0800F7FF 之前
  *     （实测基线 7.3 KB，余量极大）。
  *
  *  ⚠️ 擦写注意（CubeMX 配置清单 §4.2）：
  *     ① 写之前必须先擦除整页（1 KB）
  *     ② 擦写期间 CPU 会暂停（单 bank），所以全程关中断
  *     ③ 【不要在 100 Hz 循环里写】—— 只允许在 IDLE/CALIB 状态或命令处理里写
  *     ④ 寿命约 10000 次 —— 不要频繁写
  ******************************************************************************
  */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"   /* HAL_FLASH_* / FLASH_EraseInitTypeDef */
#include "rlca_math.h"

/** 存储布局（共 96 B：8 字节头 + 84 字节标定数据 + 2 字节校验 + 2 字节填充）
 *
 *  页内偏移  内容
 *  ---------------------------------------------------------------
 *  0x00      magic   (0x524C4341 = "RLCA")
 *  0x04      version (uint16)
 *  0x06      len     (uint16)  —— rlca_calib_t 的字节数，用于版本演进校验
 *  0x08      payload (rlca_calib_t，84 字节)
 *  0x5C      crc16   (uint16)  —— 覆盖前 0x5C 字节
 *  0x5E      pad     (uint16)  —— 凑到 4 字节边界，写 0xFFFF
 */
#define BSP_FLASH_HDR_MAGIC     0x524C4341u
#define BSP_FLASH_HDR_VERSION   1u
#define BSP_FLASH_HDR_OFF       0u
#define BSP_FLASH_PAYLOAD_OFF   8u
#define BSP_FLASH_CRC_OFF       (BSP_FLASH_PAYLOAD_OFF + (uint16_t)sizeof(rlca_calib_t))

/** 是否已从 Flash 成功加载过有效标定数据。
 *  ⚠️ 上电时必须校验 magic / version / len / crc，任一失败即返回 false，
 *     上层据此置 STAT_CALIBRATED = 0 并进入 CALIB 状态。 */
bool bsp_flash_calib_loaded(void);

/** 加载标定数据。
 *  @param out 成功时写入；失败时不修改
 *  @return true = 有一份有效的标定数据被读出
 *  @note 失败时把内部状态置为"未加载"，调用方应回退到默认值。 */
bool bsp_flash_load_calib(rlca_calib_t *out);

/** 保存标定数据（擦 1 KB 页 + 写 96 B）。
 *  @return true = 写入并回读校验通过
 *  @warning 全程关中断，耗时约 20~40 ms —— 只能在非实时路径调用 */
bool bsp_flash_save_calib(const rlca_calib_t *in);

/** 擦除标定页（恢复出厂设置用）。
 *  @warning 同样会关中断，且擦除后 bsp_flash_calib_loaded() 变 false */
bool bsp_flash_erase_calib(void);

/** 读出页内原始前 16 字节（诊断用，便于人工确认页内容）。 */
void bsp_flash_dump_header(uint8_t out16[16]);

#endif /* BSP_FLASH_H */
