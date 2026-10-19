# Changelog

本项目遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

> ⚠️ **0.x 阶段**：API 与硬件接口都可能变。`v1.0.0` 之前不保证向后兼容。

---

## [Unreleased]

### 待办
- ~~补 `LICENSE-hardware` 的 CERN-OHL-W-2.0 全文~~ ✅ 已完成
- 顶层 `README.md` 重写（英文为主）
- 固件加 Makefile/CMake，让 CI 能构建（现在只支持 Keil）
- 修正文档里的过时事实（Pi 链路波特率 1 Mbps → 500 kbps；分压比 22k → 20k；测试项数 102 → 117）

### 变更（发布范围）
- **新增机械 CAD 发布**：`hardware/mechanical/step/`（5 个零件的 STEP）与
  `stl/`（5 个 STL），并附 `mechanical/README.md`（零件表、打印参数、装配要点）
- 不发布：SolidWorks 原生文件、厂商件 3D 模型（78 MB，再分发条款不明确）、切片文件
- 移除立创 EDA 的 7 个自动备份
- `LICENSE-hardware` / `NOTICE.md` / `hardware/README.md` / 两个 README 同步更新范围

---

## [0.2.0] — 2026-10-18

**开源准备（P0 清场 + P1 许可）** —— 不改变任何运行时行为。

### 新增
- `Sim/RL/scripts/rlcat_paths.py`：统一的路径解析，**去掉了 30+ 处写死的个人目录**
- `LICENSE`（Apache-2.0）、`LICENSE-hardware`、`LICENSE-docs`、`NOTICE.md`
- `CITATION.cff`、`CHANGELOG.md`、`CONTRIBUTING.md`
- `docs/51-opensource-plan.md`：开源执行方案

### 变更
- `Sim/RL/scripts/env.sh` 改为由 `BASH_SOURCE` 反推路径，支持
  `RLCAT_ROOT` / `RLCAT_CONDA` / `RLCAT_ENV` / `RLCAT_CUDA` / `RLCAT_LEGGED_GYM` 覆盖
- `RLCA_AP_PASSWORD` 加 `#ifndef` 保护，可编译期覆盖，并加了醒目警告

### 移除
- 从版本控制移除：`MDK-ARM/_quarantine/`、`startup_*.lst`、`build.log`、
  `stm32.uvguix.<用户名>`、`.mxproject`（CubeMX 缓存，内容全是绝对路径）
- `firmware/esp32/` 下 102 个 PlatformIO `pip-unpack-*` 临时目录（28.5 MB 纯垃圾）

---

## [0.1.0] — 2026-10-18 · `v0.1-m1-pass`

**M1 地面站立通过** —— 机器人第一次在地面上稳定站住。

### 验证结果
- 到位误差 **1.23°**（判据 <3°）
- 漂移 **0.00°**（判据 <2°）
- 抖动 **0.00°** std / 0.00° 峰峰
- 非 ACTIVE 样本 **0/926**

### 修复（每条都有现场证据，详见 `docs/49-progress-review.md`）
- `rlca_q_range()` 双乘 `dir` → `dir=-1` 的 6 个关节限位被取反
  （现象：`lb_hip` / `rf_hip` 发正角完全不动）
- 几何标定表：`offset` 由常量推导，不读 flash ⇒ 不再"概率变未标定"
- 删掉 `app_init` 末尾无条件的 `s_calib_ok = bsp_flash_calib_loaded()`
- 关节顺序 leg-major ↔ group-major 换算（不修则上机第一条 action 就发错关节）
- 舵机温度坏读数过滤（>125℃ 丢弃 + 连续 5 次确认）
- 循环内 `print` 全部移除（`print` 阻塞 ⇒ 不发帧 ⇒ 链路空档 ⇒ 掉标准）
- 发帧看门狗：`link.py` 检测"开着串口却不发帧"

### 开发期放宽（⚠️ 后续要改回）
- `RLCA_LINK_LOST_MS` 80 → 150
- `RLCA_LINK_ESTOP_MS` 300 → 1200