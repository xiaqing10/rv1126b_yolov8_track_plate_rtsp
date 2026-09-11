#!/bin/sh
# RV1126B YOLO 检测推流 部署目录: /userdata/yolo/
#   用法: sh run_yolo_rtsp.sh [配置文件]
#   默认自动读 /userdata/yolo/config/yolo.conf (模型/端口/阈值等都在里面)
#
# 会停止 rkipc(独占摄像头/554)。退出(Ctrl-C)后只停 yolo, 不再自动恢复 rkipc。
# 若要恢复摄像头系统, 手动执行: rkipc -a /oem/usr/share/iqfiles &
# ======================================================================

APP_DIR=/userdata/yolo
CFG=${1:-$APP_DIR/config/yolo.conf}
IQ=/oem/usr/share/iqfiles
BIN=$APP_DIR/bin/yolo_rtsp

export LD_LIBRARY_PATH=/oem/usr/lib:/oem/lib:$LD_LIBRARY_PATH

# 启动 Web 控制面板守护(仅当未运行时才启动, 避免 /api/restart 时端口冲突)
# 该进程独立于 yolo_rtsp, stop_yolo 不会停止它, 保证网页始终可用
if ! killall -0 yolo_web 2>/dev/null; then
    echo "start yolo_web (web panel on :8080)"
    cd $APP_DIR/web && nohup ./yolo_web > web.log 2>&1 &
fi

restored=0
stop_yolo() {
    [ "$restored" = "1" ] && return 0
    restored=1
    echo
    echo "stopping yolo_rtsp ..."
    killall yolo_logrotate 2>/dev/null
    killall yolo_rtsp 2>/dev/null
    # 等待程序真正退出(它要解绑/销毁VI/VENC/RGN, 最多约10秒)
    i=0
    while pidof yolo_rtsp >/dev/null 2>&1; do
        i=$((i+1))
        [ $i -ge 20 ] && { echo "force kill ..."; killall -9 yolo_rtsp 2>/dev/null; sleep 1; break; }
        sleep 0.5
    done
    echo "yolo_rtsp stopped (rkipc NOT restarted)"
    exit 0
}
trap stop_yolo INT TERM

# 标签固定路径(与默认配置一致)
mkdir -p $APP_DIR/models/labels
cp -f $APP_DIR/models/labels/coco_80_labels_list.txt $APP_DIR/models/labels/ 2>/dev/null

echo "stop rkipc ..."
killall rkipc 2>/dev/null
sleep 2

echo "start yolo_rtsp (config: $CFG)"
# 启动日志轮转守护(超10MB轮转, 保留3份)
sh $APP_DIR/yolo_logrotate.sh &
# 有配置文件就显式传, 没有则用程序内置默认路径
if [ -f "$CFG" ]; then
    $BIN -c "$CFG"
else
    $BIN
fi

# 程序自行退出(非正常Ctrl-C)时也停 yolo
stop_yolo
