/**
  ******************************************************************************
  * @file    host_link_port_stub.c
  * @brief   PC 端测试用的 link_port_hardware() 桩（含"是否被误用"计数）。
  *
  *  ⚠️ 为什么需要它：
  *     link_pi.c 是【完全不依赖 HAL】的（I/O 全走注入的端口），
  *     但它留了一个"默认取真实硬件端口"的兜底调用 link_port_hardware()。
  *     在 PC 上链接 link_pi.c 时那个符号来自 link_port.c，
  *     而 link_port.c 会拖进整个 HAL —— 宿主机就链不上了。
  *
  *     所以这里给一个桩。所有测试都【显式注入】假端口，
  *     正确情况下这个桩一次都不该被调用；
  *     一旦被调用，测试可以用 host_link_port_uses() 断言并报错。
  ******************************************************************************
  */

#include "link_port.h"

static int s_uses = 0;

/** 桩被调用的次数。测试结尾应断言为 0。 */
int host_link_port_uses(void);

const rlca_link_port_t *link_port_hardware(void);
const rlca_link_port_t *link_port_hardware(void)
{
    s_uses++;
    /* 返回 NULL 是最安全的选择：link_pi 的每个入口都会检查端口为空，
     * 于是"忘了注入"表现为"什么都没发生"，测试会因为断言不成立而失败，
     * 而不是静默地连上不存在的外设。 */
    return NULL;
}

int host_link_port_uses(void)
{
    return s_uses;
}
