#!/bin/sh
# ============================================================
# package_yolo.sh - 在【编译机】上把运行所需打成 tar 包
#   用法:  sh deploy/package_yolo.sh [输出路径]
#          默认输出: ./yolo_deploy.tar   (可用参数 1 指定路径)
#   产出:  一个顶层为 yolo/ 的 tar 包, 含板端完整部署目录
#   部署:  在目标板执行
#            tar -xvf yolo_deploy.tar -C /userdata
#          即可得到 /userdata/yolo/ 完整部署目录, 然后
#            cd /userdata/yolo && sh run_yolo_rtsp.sh
#
#   打包内容(按板端 /userdata/yolo 布局):
#     yolo/bin/                      全部板端可执行文件
#        yolo_rtsp                   主程序
#        yolo_img_test               测试图片: 单张静态图 YOLOv8 推理
#        car_img_test                测试车牌: 单张静态图 车牌/车辆识别
#        stream_demo                 (可选) 采集编码 RTSP 骨架 demo
#     yolo/web/yolo_web              Web 面板
#     yolo/config/yolo.conf          运行配置
#     yolo/models/*.rknn             全部模型
#     yolo/models/labels/            类别标签
#     yolo/run_yolo_rtsp.sh          启动脚本
#     yolo/yolo_logrotate.sh         日志轮转
# ============================================================

set -e

# 定位仓库根目录 (本脚本在 deploy/ 下)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

OUT="${1:-$ROOT/yolo_deploy.tar}"

# 板端 bin/ 下要打包的可执行文件 (全部来自 make all 产物)
BIN_REQUIRED="yolo_rtsp"
BIN_OPTIONAL="yolo_img_test car_img_test stream_demo"

# ---- 校验运行所需的文件都在 ----
err() { echo "错误: $1"; exit 1; }
for b in $BIN_REQUIRED; do
    [ -x "$b" ] || err "缺少可执行文件 $b (请先 make all)"
done
[ -x web/yolo_web ]       || err "缺少可执行文件 web/yolo_web (请先 make all)"
[ -f config/yolo.conf ]   || err "缺少 config/yolo.conf"
[ -d models ]              || err "缺少 models/ 目录"
[ -f run_yolo_rtsp.sh ]   || err "缺少 run_yolo_rtsp.sh"
[ -f yolo_logrotate.sh ]  || err "缺少 yolo_logrotate.sh"
if ! ls models/*.rknn >/dev/null 2>&1; then err "models/ 下没有 .rknn 模型"; fi

# ---- 组装临时目录 (yolo/ 前缀, 按板端布局) ----
STAGE="$(mktemp -d /tmp/yolo_pkg.XXXXXX)"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$STAGE/yolo/bin" \
         "$STAGE/yolo/web" \
         "$STAGE/yolo/config" \
         "$STAGE/yolo/models/labels"

# 必需可执行文件
for b in $BIN_REQUIRED; do
    cp -f "$b" "$STAGE/yolo/bin/$b"
    chmod 755 "$STAGE/yolo/bin/$b"
done
# 可选测试/演示可执行文件: 存在才打包, 缺失只警告不中断
for b in $BIN_OPTIONAL; do
    if [ -x "$b" ]; then
        cp -f "$b" "$STAGE/yolo/bin/$b"
        chmod 755 "$STAGE/yolo/bin/$b"
        echo "已打包测试工具 $b"
    else
        echo "警告: 未找到 $b, 跳过 (如需请先 make all)"
    fi
done
cp -f web/yolo_web        "$STAGE/yolo/web/yolo_web"
cp -f config/yolo.conf    "$STAGE/yolo/config/yolo.conf"
cp -f models/*.rknn       "$STAGE/yolo/models/"
cp -f models/labels/*.txt "$STAGE/yolo/models/labels/"
cp -f run_yolo_rtsp.sh    "$STAGE/yolo/run_yolo_rtsp.sh"
cp -f yolo_logrotate.sh   "$STAGE/yolo/yolo_logrotate.sh"

chmod 755 "$STAGE/yolo/run_yolo_rtsp.sh" \
          "$STAGE/yolo/yolo_logrotate.sh" \
          "$STAGE/yolo/web/yolo_web"

# ---- 打 tar (不压缩) ----
echo "打包到 $OUT ..."
tar cf "$OUT" -C "$STAGE" yolo

echo "打包完成: $OUT"
ls -lh "$OUT"
echo
echo "部署到板端:"
echo "  tar -xvf $(basename "$OUT") -C /userdata"
echo "  cd /userdata/yolo && sh run_yolo_rtsp.sh"
