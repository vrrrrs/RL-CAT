"""把 07_eval.py 录下的 PNG 帧序列合成 mp4/GIF

用法（缺省路径由 rlcat_paths.py 解析，不需要改代码）:
    python scripts/09_frames_to_video.py
    python scripts/09_frames_to_video.py --frames <你的帧目录> --out <输出目录> --fps 25
"""
import argparse
import glob
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rlcat_paths as P  # noqa: E402  ⭐ 统一路径解析，不写死个人目录


def ffmpeg_exe():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def make_mp4(exe, frame_dir, out_path, fps):
    pattern = os.path.join(frame_dir, "%05d.png")
    cmd = [exe, "-y", "-loglevel", "error",
           "-framerate", str(fps), "-i", pattern,
           "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
           out_path]
    subprocess.run(cmd, check=True)
    return out_path


def make_gif(frame_dir, out_path, fps, width=560, native_fps=25):
    """native_fps: 抓帧时的原始帧率（07_eval 每 4 步抓 1 帧，100Hz -> 25fps）。
    目标 fps 更低时按比例抽帧，避免 GIF 过大。"""
    from PIL import Image
    files = sorted(glob.glob(os.path.join(frame_dir, "*.png")))
    if not files:
        return None
    stride = max(1, int(round(native_fps / max(fps, 1))))
    files = files[::stride]
    h = int(round(width * 9 / 16))
    frames = [Image.open(f).convert("RGB").resize((width, h)) for f in files]
    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    return out_path


def make_combined_gif(subdirs, out_path, fps, width=560, native_fps=25):
    """把多个速度的帧首尾拼成一个 GIF，方便一次看完"""
    from PIL import Image
    h = int(round(width * 9 / 16))
    stride = max(1, int(round(native_fps / max(fps, 1))))
    all_frames = []
    for d in subdirs:
        files = sorted(glob.glob(os.path.join(d, "*.png")))[::stride]
        all_frames += [Image.open(f).convert("RGB").resize((width, h)) for f in files]
    if not all_frames:
        return None
    all_frames[0].save(out_path, save_all=True, append_images=all_frames[1:],
                       duration=int(1000 / fps), loop=0, optimize=True)
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", default=P.EVAL_FRAMES_DIR,
                    help="07_eval.py 的 --out 目录（缺省 = $RLCAT_FRAMES）")
    ap.add_argument("--out", default=P.VIDEOS_DIR,
                    help="视频输出目录（缺省 = $RLCAT_VIDEOS）")
    ap.add_argument("--fps", type=int, default=25, help="07_eval 每 4 步抓 1 帧，100Hz -> 25fps")
    ap.add_argument("--gif", action="store_true", help="同时生成 gif")
    ap.add_argument("--gif-only", action="store_true", help="只生成 gif，不生成 mp4")
    ap.add_argument("--gif-fps", type=int, default=12, help="gif 帧率（越低文件越小）")
    ap.add_argument("--combine", action="store_true",
                    help="额外把所有速度拼成一个合集 gif")
    opt = ap.parse_args()

    os.makedirs(opt.out, exist_ok=True)
    subs = sorted(d for d in glob.glob(os.path.join(opt.frames, "*"))
                  if os.path.isdir(d))
    if not subs:
        print("[FAIL] %s 下没有子目录（先跑 07_eval.py --record）" % opt.frames)
        return 1

    exe = ffmpeg_exe()
    if exe is None:
        print("[警告] 没有 ffmpeg，只能生成 gif")
    made = []
    for d in subs:
        tag = os.path.basename(d)
        n = len(glob.glob(os.path.join(d, "*.png")))
        if n == 0:
            print("  跳过 %s（无帧）" % tag)
            continue
        if exe and not opt.gif_only:
            out = os.path.join(opt.out, "%s.mp4" % tag)
            make_mp4(exe, d, out, opt.fps)
            size = os.path.getsize(out) / 1e6
            print("  %-10s %4d 帧 -> %s (%.1f MB)" % (tag, n, out, size))
            made.append(out)
        if opt.gif or opt.gif_only or not exe:
            out = os.path.join(opt.out, "%s.gif" % tag)
            make_gif(d, out, opt.gif_fps)
            print("  %-10s %4d 帧 -> %s (%.1f MB)"
                  % (tag, n, out, os.path.getsize(out) / 1e6))
            made.append(out)

    if opt.combine:
        out = os.path.join(opt.out, "ALL_speeds.gif")
        make_combined_gif(subs, out, opt.gif_fps)
        print("  合集        -> %s (%.1f MB)" % (out, os.path.getsize(out) / 1e6))
        made.append(out)

    print("\n完成，共 %d 个视频：%s" % (len(made), opt.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
