#!/bin/sh
# ============================================================
# deploy_yolo.sh - 在【目标板】上解压部署 (配合编译机的 package_yolo.sh)
#   用法:  sh deploy_yolo.sh [tar包路径]
#          默认从当前目录找 yolo_deploy.tar; 可用参数 1 指定路径
#   效果:  把包解压到 /userdata/, 得到 /userdata/yolo/ 完整部署目录
#   等价手工命令:
#           tar -xvf yolo_deploy.tar -C /userdata
#           cd /userdata/yolo && sh run_yolo_rtsp.sh
# ============================================================

set -e

TAR="${1:-yolo_deploy.tar}"

[ -f "$TAR" ] || { echo "错误: 找不到 $TAR, 请先拷贝打包文件到本目录"; exit 1; }

echo "停止正在运行的 yolo 进程 ..."
killall yolo_web yolo_rtsp 2>/dev/null || true
sleep 1

echo "解压 $TAR 到 /userdata/ ..."
tar -xvf "$TAR" -C /userdata

echo
echo "部署完成. 启动:"
echo "  cd /userdata/yolo && sh run_yolo_rtsp.sh"
