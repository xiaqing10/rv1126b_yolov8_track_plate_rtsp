#!/bin/sh
# yolo_rtsp 日志轮转守护脚本
#   当 autostart.log 超过 MAXSIZE 字节时, 复制当前内容为带时间戳的历史文件,
#   并截断主日志(进程 fd 仍指向原文件, 可继续写入, 即 copytruncate)。
#   只保留最近 KEEP 个历史文件, 更早的自动删除。
# 用法: sh yolo_logrotate.sh &   (由 run_yolo_rtsp.sh 自动启动)
LOG=/userdata/yolo/autostart.log
MAXSIZE=10485760   # 10MB
KEEP=3
while :; do
    if [ -f "$LOG" ]; then
        sz=$(wc -c < "$LOG" 2>/dev/null)
        if [ -n "$sz" ] && [ "$sz" -gt "$MAXSIZE" ]; then
            ts=$(date +%Y%m%d_%H%M%S)
            # 删除超出保留份数的旧历史
            ls -t "$LOG".[0-9]* 2>/dev/null | tail -n +$((KEEP+1)) | xargs rm -f 2>/dev/null
            # copytruncate: 复制当前内容, 再截断主日志
            cp "$LOG" "$LOG.$ts" 2>/dev/null
            : > "$LOG"
        fi
    fi
    sleep 30
done
