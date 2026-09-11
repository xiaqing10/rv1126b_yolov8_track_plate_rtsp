#!/usr/bin/env python3
"""TCP + NDJSON 解析客户端.

连接 yolo_rtsp 的 TCP 输出端口 (默认 9000), 按行 (\\n) 解析 NDJSON 消息:
  - frame      每帧跟踪结果.  cls=数字类别, box=[x0,y0,x1,y1] 为 0-1 归一化 (整幅-AI 基准)
  - detection  最近一次车牌/车辆原始识别. box 归一化到整幅 (fw x fh)
  - plate_adopted  (若启用) 已采信车牌

用法:
  python3 tools/tcp_recv.py [--host HOST] [--port PORT] [--show frame,detection]
                            [--json] [--stats]

示例:
  python3 tools/tcp_recv.py                        # 打印所有 frame/detection
  python3 tools/tcp_recv.py --show detection       # 只看车牌识别结果
  python3 tools/tcp_recv.py --json                 # 原样打印原始 JSON 行
  python3 tools/tcp_recv.py --stats                # 每秒统计消息/目标数
"""
import argparse
import json
import socket
import sys
import time
from collections import Counter


CLS_NAMES = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
    4: "airplane", 5: "bus", 6: "train", 7: "truck",
}


def parse_args():
    p = argparse.ArgumentParser(description="yolo_rtsp TCP/NDJSON 解析客户端")
    p.add_argument("--host", default="127.0.0.1", help="TCP 服务器地址")
    p.add_argument("--port", type=int, default=9000, help="TCP 端口")
    p.add_argument("--show", default="frame,detection",
                   help="要显示的消息类型, 逗号分隔 (frame/detection/plate_adopted), 默认 frame,detection")
    p.add_argument("--json", action="store_true", help="原样打印原始 JSON 行")
    p.add_argument("--stats", action="store_true", help="每秒输出统计摘要")
    return p.parse_args()


def fmt_box(box):
    """把 0-1 归一化 box 转成百分比字符串."""
    if not box or len(box) < 4:
        return "-"
    x0, y0, x1, y1 = box
    return f"[{x0*100:.1f}%,{y0*100:.1f}%→{x1*100:.1f}%,{y1*100:.1f}%]"


def fmt_plate(r):
    parts = []
    if r.get("plate"):
        parts.append(f"牌={r['plate']}")
    if r.get("plate_color"):
        parts.append(f"牌色={r['plate_color']}")
    if r.get("car_color"):
        parts.append(f"车色={r['car_color']}")
    return " ".join(parts)


def show_frame(obj):
    objs = obj.get("objs", [])
    if not objs:
        return None
    lines = [f"  [frame seq={obj.get('seq')}] {len(objs)} obj:"]
    for o in objs:
        cls = o.get("cls", 0)
        name = CLS_NAMES.get(cls, f"cls{cls}")
        tid = o.get("track_id")
        conf = o.get("conf", 0)
        lines.append(f"    - {name:<9} id={tid:<3} conf={conf:.2f} box={fmt_box(o.get('box'))}")
    return "\n".join(lines)


def show_detection(obj):
    objs = obj.get("objs", [])
    if not objs:
        return None
    lines = [f"  [detection fw={obj.get('fw')}x{obj.get('fh')}] {len(objs)} obj:"]
    for r in objs:
        lines.append(f"    - class={r.get('class')} conf={r.get('conf', 0):.2f} "
                     f"box={fmt_box(r.get('box'))} {fmt_plate(r)}")
    return "\n".join(lines)


def main():
    args = parse_args()
    wanted = {s.strip() for s in args.show.split(",") if s.strip()}

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.host, args.port))
    except OSError as e:
        print(f"[ERROR] 连接 {args.host}:{args.port} 失败: {e}", file=sys.stderr)
        return 1
    sock.settimeout(None)
    print(f"[OK] 已连接 {args.host}:{args.port}, 监听消息: {sorted(wanted)}")

    buf = b""
    f = sock.makefile("rb")
    stats = Counter()
    last_stats = time.time()

    try:
        for raw in f:
            buf += raw
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    obj = json.loads(line.decode("utf-8"))
                except (ValueError, UnicodeDecodeError) as e:
                    print(f"[WARN] 解析失败: {e}: {line[:80]!r}")
                    continue

                t = obj.get("type")
                stats["total"] += 1
                stats[t] += 1
                stats["objs"] += len(obj.get("objs", []))

                if args.json:
                    print(line.decode("utf-8", "replace"))
                    continue
                if t not in wanted:
                    continue

                if t == "frame":
                    out = show_frame(obj)
                elif t == "detection":
                    out = show_detection(obj)
                elif t == "plate_adopted":
                    out = f"  [plate_adopted] {fmt_plate(obj)} box={fmt_box(obj.get('box'))}"
                else:
                    out = f"  [{t}] {json.dumps(obj, ensure_ascii=False)}"
                if out:
                    print(out)

                if args.stats:
                    now = time.time()
                    if now - last_stats >= 1.0:
                        print(f"  -- 1s: total={stats['total']} frame={stats['frame']} "
                              f"detection={stats['detection']} objs={stats['objs']} --")
                        stats.clear()
                        last_stats = now
    except KeyboardInterrupt:
        print("\n[退出] 中断")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
