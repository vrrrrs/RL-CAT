# docs/pictures/

放 README 与文档引用的图片。**图片不适用于 git 的场景请用 Release 附件。**

## 待补（⚠️ 阻塞 README 的完整度）

| 文件 | 内容 | 要求 |
|---|---|---|
| `stand.gif` | ✅ **已完成**（真机，420 宽 / 10 fps / 2.7 MB） |    |
| `architecture.svg` | 系统架构图 | ✅ **已完成**（手写 SVG，已嵌入 README） |
| `robot.jpg`、`robot2.jpg` | ✅ **已完成**（1600 宽，169 / 131 KB） |    |
| `joint-frame.svg` | 12 关节编号与正方向 | ⬜ **未做** —— 需确认每个关节轴向 |

## 生成 GIF 的参考命令

```bash
# 从视频
ffmpeg -i stand.mp4 -vf "fps=12,scale=640:-1:flags=lanczos" -loop 0 stand.gif

# 如果体积超了，降色板
ffmpeg -i stand.mp4 -vf "fps=10,scale=560:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" -loop 0 stand.gif
```

⚠️ 视频/GIF **必须诚实**：如果只能吊着站，就拍吊着的；不要剪成"看起来在走"。
---

## 已完成的产出（2026-10-18）

| 文件 | 来源 | 规格 |
|---|---|---|
| `stand.gif` | `素材/video_20261018_230328.mp4` | 420×236，10 fps，68 帧，**2.7 MB** |
| `sim_all_speeds.gif` | `Sim/RL/videos/ALL_speeds.gif` | 340×191，8 fps，150 帧，**1.5 MB** |
| `robot.jpg` | `素材/IMG_20261018_230501.jpg` | 1600×1200，**169 KB** |
| `robot2.jpg` | `素材/IMG_20261018_230509.jpg` | 1600×1200，**131 KB** |
| `architecture.svg` | 手写 | 1000×620，**7 KB** |

**原始素材（`素材/`、`Sim/RL/videos/`）不进 git** —— 已在 `.gitignore` 里。
这里只放压缩后的 web 用图，仓库体积可控。

⚠️ `sim_all_speeds.gif` 是**仿真**画面（Isaac Gym 里走路），
README 里已明确标注它与真机的区别。**不要把它放到 README 顶部当封面。**