# hardware/

本目录说明硬件资料的发布范围。

---

## 1 · 已发布

| 路径 | 内容 | 格式 |
|---|---|---|
| `mechanical/step/` | 零件 CAD：`base_link`、`thigh`、`calf`、`hip1`、`hip2` | STEP AP214 |
| `mechanical/stl/` | 可打印网格：`thigh`、`calf`、`hip1`、`hip2`、`link` | STL（二进制） |
| `mechanical/URDF/robot.urdf` | 实机运动学与动力学描述：关节轴、限位、连杆质量与惯量 | URDF |
| `mechanical/README.md` | 零件清单、**打印参数**、装配要点、已知缺件 | Markdown |
| `electronics/RL-CAT.eprj2` | 立创 EDA 工程（原理图与 PCB） | 立创 EDA |

**打印参数（PLA Basic、立方体填充 30%、五层墙、交替额外层墙、底部壳体 4 层、树状支撑）
记录于 `mechanical/README.md` §3。**

---

## 2 · 未发布

| 路径 | 内容 | 不发布的原因 |
|---|---|---|
| `mechanical/装配体/` | SolidWorks 装配体与版本备份（216 MB） | 二进制格式，不适合版本控制；STEP 导出已包含同等几何 |
| `mechanical/零件/` | SolidWorks 零件与厂商模型（60 MB） | 同上 |
| `mechanical/Opensource/` | 原生文件与厂商件 3D 模型（78 MB） | 厂商模型（PCB 封装、树莓派、舵机）的再分发条款不明确，且与复现无关 |
| `mechanical/参数.xlsx`、`质量属性.txt` | 尺寸参数与质量属性原始数据 | 已并入 `robot.urdf` 与 `mechanical/README.md` |
| 切片文件（`.gcode`、`.3mf`） | 打印机相关 | 参数已在 `mechanical/README.md` §3 给出，可自行切片 |

以上路径均写入 [`.gitignore`](../.gitignore)。

---

## 3 · 关节限位以哪一个 URDF 为准

| URDF | 路径 | thigh 上限 | 用途 |
|---|---|---|---|
| 实机 | `mechanical/URDF/robot.urdf` | **30°** | 固件限位与 `rl_run --clamp-real` |
| 仿真 | `Sim/legged_gym/resources/robots/rlcat/urdf/robot_sim.urdf` | **90°** | 训练时为摆腿预留空间 |

两者不一致，是本项目最重要的已知差距（G1）。策略在仿真中的指令最大达到 46.9°，
因此真机首次运行必须使用 `pi/rl_run.py --clamp-real` 将目标夹取至 30°。

详见 [`docs/48-sim2real-gaps.md`](../docs/48-sim2real-gaps.md) §G1。

---

## 4 · 复用其他机械结构

固件与软件不依赖机械的具体尺寸，仅依赖十二个关节的编号、方向与限位
（`firmware/stm32/Core/Inc/rlcat_config.h`）。更换机械结构时，同步更新该文件与
`robot.urdf` 即可，固件与软件无需改动。
