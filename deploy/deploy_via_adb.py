#!/usr/bin/env python3
# ============================================================
# deploy_via_adb.py - 通过 adb 把编译机打好的 tar 部署到目标板
#   用法:
#     python deploy_via_adb.py [tar路径]
#          默认 tar: 同目录/本机上的 yolo_deploy.tar (可用参数 1 指定)
#   流程: 把编译机打好的 tar 推送到板端 /userdata/, 再调用 deploy_yolo.sh 解压
#   说明: 打包在编译机完成(见 deploy/package_yolo.sh), 本脚本只负责"推+解压"
# ============================================================
import subprocess, sys, os

DEP_SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "deploy_yolo.sh")
TAR    = os.path.join(os.getcwd(), "yolo_deploy.tar")
ADB    = "adb"

def run(cmd):
    print("+", " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True)
    out = r.stdout.decode("utf-8", errors="replace")
    err = r.stderr.decode("utf-8", errors="replace")
    if out.strip(): print(out)
    if err.strip(): print(err, file=sys.stderr)
    return r.returncode

def main():
    global TAR
    if len(sys.argv) > 1:
        TAR = sys.argv[1]
    if not os.path.exists(TAR):
        print("错误: 找不到", TAR); return 1
    print("=== 部署 tar 到目标板:", TAR, "===")
    # 推 tar 到板端
    if run([ADB, "push", TAR, "//userdata/yolo_deploy.tar"]) != 0: return 1
    # 推 deploy 脚本并执行
    if run([ADB, "push", DEP_SH, "//data/local/tmp/deploy_yolo.sh"]) != 0: return 1
    return run([ADB, "shell", "sh /data/local/tmp/deploy_yolo.sh /userdata/yolo_deploy.tar"])

if __name__ == "__main__":
    sys.exit(main())
