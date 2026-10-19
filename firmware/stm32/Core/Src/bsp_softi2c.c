/**
  ******************************************************************************
  * @file    bsp_softi2c.c
  * @brief   软件 I2C 主机实现（开漏 + DWT 定时）。
  ******************************************************************************
  */

#include "bsp_softi2c.h"
#include "bsp_tick.h"
#include "main.h"          /* SCL_Pin / SDA_Pin / SCL_GPIO_Port / SDA_GPIO_Port */

/*==============================================================================
 * 底层位操作
 *============================================================================*/

static inline void scl_release(void)
{
    HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);   /* 开漏：置 1 = 释放 */
}

static inline void scl_low(void)
{
    HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
}

static inline void sda_release(void)
{
    HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);
}

static inline void sda_low(void)
{
    HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);
}

/** ⚠️ 读电平必须读 IDR，不能读 ODR（开漏下 ODR=1 只表示"释放"） */
static inline bool sda_read(void)
{
    return HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin) == GPIO_PIN_SET;
}

static inline bool scl_read(void)
{
    return HAL_GPIO_ReadPin(SCL_GPIO_Port, SCL_Pin) == GPIO_PIN_SET;
}

static inline void i2c_delay(void)
{
    bsp_delay_us(BSP_I2C_HALF_US);
}

/*==============================================================================
 * 时序原语
 *============================================================================*/

static void i2c_start(void)
{
    sda_release();
    scl_release();
    i2c_delay();
    sda_low();          /* SCL 高时 SDA 下降 = START */
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_low();
    scl_release();
    i2c_delay();
    sda_release();      /* SCL 高时 SDA 上升 = STOP */
    i2c_delay();
}

/** 写一位，返回后 SCL 为低。 */
static void i2c_write_bit(bool bit)
{
    if (bit) {
        sda_release();
    } else {
        sda_low();
    }
    i2c_delay();
    scl_release();
    i2c_delay();
    scl_low();
    i2c_delay();
}

static bool i2c_read_bit(void)
{
    sda_release();      /* 主机释放，让从机驱动 */
    i2c_delay();
    scl_release();
    i2c_delay();
    const bool b = sda_read();
    scl_low();
    i2c_delay();
    return b;
}

/** 写一字节，返回从机的 ACK（true = 被应答）。 */
static bool i2c_write_byte(uint8_t b)
{
    for (uint8_t i = 0u; i < 8u; ++i) {
        i2c_write_bit((b & 0x80u) != 0u);
        b = (uint8_t)(b << 1);
    }
    return !i2c_read_bit();       /* ACK = SDA 被从机拉低 */
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t b = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) {
        b = (uint8_t)(b << 1);
        if (i2c_read_bit()) {
            b |= 1u;
        }
    }
    i2c_write_bit(!ack);          /* ack=true → 写 0（ACK）；false → 写 1（NACK） */
    return b;
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void bsp_softi2c_init(void)
{
    /* GPIO 模式已由 CubeMX 的 MX_GPIO_Init() 配成开漏 + 高速 + 无内部上拉。
     * 这里只把总线释放成空闲态（两根线都为高）。 */
    sda_release();
    scl_release();
    bsp_delay_us(10u);            /* 等上拉把总线拉高 */
}

bool bsp_softi2c_bus_ok(void)
{
    sda_release();
    scl_release();
    bsp_delay_us(20u);
    return sda_read() && scl_read();
}

void bsp_softi2c_bus_recover(void)
{
    /* 从机若在传输中途复位，可能一直拉着 SDA。发 9 个时钟让它把剩下的位吐完。
     * ⚠️ 全程用"软件模拟时钟"，不依赖 I2C 外设。 */
    sda_release();

    for (uint8_t i = 0u; i < 9u; ++i) {
        scl_low();
        i2c_delay();
        scl_release();
        i2c_delay();
    }
    /* 补一个标准 STOP：SCL 高时 SDA 由低变高 */
    sda_low();
    i2c_delay();
    scl_release();
    i2c_delay();
    sda_release();
    i2c_delay();
}

bool bsp_softi2c_probe(uint8_t dev)
{
    i2c_start();
    const bool ack = i2c_write_byte((uint8_t)(dev << 1));
    i2c_stop();
    return ack;
}

bool bsp_softi2c_write_reg(uint8_t dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
    if ((data == NULL) && (len > 0u)) {
        return false;
    }

    i2c_start();
    if (!i2c_write_byte((uint8_t)(dev << 1))) {    /* 地址 + 写 */
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return false;
    }
    for (uint8_t i = 0u; i < len; ++i) {
        if (!i2c_write_byte(data[i])) {
            i2c_stop();
            return false;
        }
    }
    i2c_stop();
    return true;
}

bool bsp_softi2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    if ((data == NULL) || (len == 0u)) {
        return false;
    }

    /* 第一阶段：写寄存器地址 */
    i2c_start();
    if (!i2c_write_byte((uint8_t)(dev << 1))) {
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return false;
    }

    /* 第二阶段：RESTART + 读 */
    i2c_start();
    if (!i2c_write_byte((uint8_t)((dev << 1) | 0x01u))) {
        i2c_stop();
        return false;
    }

    for (uint8_t i = 0u; i < len; ++i) {
        /* 最后一个字节回 NACK，告诉从机"我读完了" */
        data[i] = i2c_read_byte(i < (uint8_t)(len - 1u));
    }
    i2c_stop();
    return true;
}
