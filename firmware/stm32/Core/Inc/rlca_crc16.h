/**
  ******************************************************************************
  * @file    rlca_crc16.h
  * @brief   CRC-16/CCITT-FALSE，用于树莓派通信帧。
  *
  *  参数（协议规范 §5）：
  *    多项式 0x1021，初值 0xFFFF，输入/输出均不反射，异或输出 0x0000
  *
  *  ⚠️ 标准测试向量：crc16("123456789") == 0x29B1
  *     两端（STM32 与树莓派）必须用同一个向量验证。
  *
  *  本模块是【纯逻辑】：不依赖 HAL，可在 PC 上用 gcc 单测。
  ******************************************************************************
  */

#ifndef RLCA_CRC16_H
#define RLCA_CRC16_H

#include <stdint.h>
#include <stddef.h>

/** 计算 CRC-16/CCITT-FALSE。
 *  @param data 数据首地址（可为 NULL 仅当 len == 0）
 *  @param len  字节数
 *  @return 16 位校验值 */
uint16_t rlca_crc16(const uint8_t *data, uint16_t len);

/** 增量式 CRC：便于逐段计算（例如边收边算）。
 *  @param crc  上一段的返回值，首次调用传 RLCA_CRC16_INIT
 *  @param data 本段数据
 *  @param len  本段字节数 */
uint16_t rlca_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len);

#define RLCA_CRC16_INIT  0xFFFFu

#endif /* RLCA_CRC16_H */
