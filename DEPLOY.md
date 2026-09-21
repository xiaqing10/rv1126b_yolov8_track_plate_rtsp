# 部署与编译指南

本文档说明如何在 RV1126B IPC 开发板上编译并部署 `yolo_rtsp`（目标检测 + 车牌/车辆识别 + RTSP 推流）与 `yolo_web`（Web 控制面板），以及如何快速部署到另一台设备。

## 一、整体结构

| 目录/文件 | 作用 | 是否入库 |
|-----------|------|----------|
| `src/`, `include/` | 源码 | 是 |
| `Makefile` | 交叉编译 | 是 |
| `config/yolo.conf` | 运行配置模板 | 是 |
| `labels/kezhu_labels.txt` | 10 类检测标签 | 是 |
| `run_yolo_rtsp.sh` | 板端启动脚本 | 是 |
| `yolo_logrotate.sh` | 日志轮转脚本 | 是 |
| `web/yolo_web.c` + `web/README.md` | Web 面板源码/文档 | 是 |
| `deploy/` | 打包/部署工具脚本 | 是 |
| `lib/`, `models/`, `*.rknn` | 预编译运行库 / 模型 | 否（见下） |

> **运行库不入库**：板厂出厂系统已在 `/oem/usr/lib`、`/usr/lib`、`/lib` 提供全部运行依赖（`librockit.so`、`librknnrt.so`、`librockchip_mpp.so.1`、`librga.so`、`librkaiq.so`、`librksysutils.so`、`libdrm.so.2`、`libasound.so.2`、glibc 等），**部署时无需携带任何额外库**。
>
> **模型不入库**：`.rknn` 模型体积大，需单独从构建目录拷贝。

## 二、交叉编译

在编译服务器上（含 RV1126B SDK 工具链）执行：

```sh
source /root/sdk/env-rv1126b.sh      # 设置 CC=arm-rockchip1240-linux-gnueabihf-gcc
cd /root/sdk/yolo_rtsp_app
make all
```

产物：
- `yolo_rtsp`（根目录）→ 部署到板端 `bin/yolo_rtsp`
- `web/yolo_web`（`web/` 下）→ 部署到板端 `web/yolo_web`
- `stream_demo`, `yolo_img_test`（测试程序，可不部署）

> 只编译 Web 面板：`make web/yolo_web`。
> 清理：`make clean`。

## 三、板端部署目录

目标板固定部署目录 `/userdata/yolo/`，完整布局如下：

```
/userdata/yolo/
├── bin/yolo_rtsp                     # 主程序
├── web/yolo_web                      # Web 面板守护进程
├── config/yolo.conf                  # 运行配置
├── models/                           # 全部 .rknn 模型 + 标签
│   ├── detect.rknn                   #   车牌/车辆/两轮车检测
│   ├── plate_rec_color.rknn          #   车牌号 + 车牌颜色识别
│   ├── car_recognition.rknn          #   车身颜色识别
│   ├── lprnet.rknn                   #   备用: 纯车牌号识别
│   ├── yolov8s_80_rv1126b.rknn       #   主 YOLOv8 检测
│   └── labels/kezhu_labels.txt
├── run_yolo_rtsp.sh                  # 启动脚本
└── yolo_logrotate.sh                 # 日志轮转
```

配置文件 `config/yolo.conf` 中的模型路径已指向 `/userdata/yolo/models/*.rknn`，部署后无需修改（除非更换识别引擎或模型）。

## 四、快速部署到新设备

同一型号的 RV1126B 设备，运行库由出厂系统自带，**只需拷贝 `bin`/`web`/`config`/`models`/`labels` 五个部分 + 两个脚本**。

> ⚠️ **板端是 BusyBox tar，不支持 `-z` 参数**，压缩必须用 `tar | gzip` 管道，解压用 `gzip -dc | tar xf`。仓库内 `deploy/` 已提供现成脚本。

### 方式一：git 拉取源码再构建/拷贝（每次改代码用）

```sh
# 1. 在编译服务器拉取最新代码
cd /root/sdk/yolo_rtsp_app && git pull origin master

# 2. 构建
source /root/sdk/env-rv1126b.sh && make all

# 3. 拷贝到新设备 (scp 或 adb push)
scp yolo_rtsp     root@<newboard>:/userdata/yolo/bin/yolo_rtsp
scp web/yolo_web  root@<newboard>:/userdata/yolo/web/yolo_web
scp run_yolo_rtsp.sh yolo_logrotate.sh root@<newboard>:/userdata/yolo/
```

### 方式二：编译机打包 → 板端直接 tar -xvf（推荐，适合批量/完整环境）

在**编译机**上把可执行文件、models、config、web、bin 及脚本打成**一个 tar 包**，拷到板端后**直接 `tar -xvf`** 即完成部署：

```sh
# 1. 编译机: 打包 (顶层为 yolo/, 即板端 /userdata/yolo 的完整布局)
sh deploy/package_yolo.sh            # 产出 ./yolo_deploy.tar (不压缩)

# 2. 拷贝 tar 到板端 (scp / adb / U盘 均可)
scp yolo_deploy.tar root@<newboard>:/

# 3. 板端: 直接解压到 /userdata (得到 /userdata/yolo/...)
tar -xvf yolo_deploy.tar -C /userdata

# 4. 启动
cd /userdata/yolo && sh run_yolo_rtsp.sh
```

也提供板端辅助脚本（等价第 3 步，多一层"先停进程"）：

```sh
sh deploy/deploy_yolo.sh /path/to/yolo_deploy.tar
```

> 打包内容：`bin/`（`yolo_rtsp` + 测试工具 `yolo_img_test`/`car_img_test`/`stream_demo`）、`web/yolo_web`、`config/yolo.conf`、`models/*.rknn`+`models/labels/`、`run_yolo_rtsp.sh`、`yolo_logrotate.sh`。仓库根目录的 `yolo_rtsp`、测试可执行文件会自动归位到包内 `yolo/bin/`。测试工具若未编译则跳过并给出警告（`yolo_rtsp`/`yolo_web` 为必需）。

### 方式三：Windows 上经 adb 一条龙（推荐，无需板端网络）

`deploy/deploy_via_adb.py` 把编译机打好的 tar 直接推到 adb 连接的板端并解压：

```sh
# 编译机打好包后, 板端通过 USB/adb 连上, 执行:
python deploy/deploy_via_adb.py            # 用默认 ./yolo_deploy.tar
python deploy/deploy_via_adb.py D:\...\yolo_deploy.tar   # 指定包路径
```

### 新设备上启动

```sh
chmod +x /userdata/yolo/bin/yolo_rtsp /userdata/yolo/web/yolo_web
cd /userdata/yolo && sh run_yolo_rtsp.sh
```

启动脚本会自动：
- 启动 `yolo_web`（Web 面板，`http://<board-ip>:8080`，幂等判断不重复启）
- 停止占用摄像头的 `rkipc`
- 按 `config/yolo.conf` 启动 `yolo_rtsp`（RTSP 流 `rtsp://<board-ip>:8554/live/0`）
- 挂上 `yolo_logrotate.sh` 日志轮转

## 五、运行配置要点

- 摄像头模式：`config/yolo.conf` 中 `video` 行保持注释；文件回放则填入裸流 `.h264/.h265` 路径。
- 车牌/车辆识别：`car_enable = 1` 启用；`car_plate_type = rec`（车牌号+颜色）或 `lprnet`（仅车牌号）。
- ROI 需按实际安装场景调整（`car_roi_x/y/w/h`，源帧 4K 坐标 0~3840 × 0~2160）。
- 识别引擎/模型路径：`car_detect_model` / `car_plate_model` / `car_car_model`。

## 六、Web 面板

- 页面：`http://<board-ip>:8080`
- 接口：`GET /api/state`、`POST /api/start|stop|restart|save`、`GET /api/plates`（SSE 实时车牌）
- 详细说明见 `web/README.md`。
