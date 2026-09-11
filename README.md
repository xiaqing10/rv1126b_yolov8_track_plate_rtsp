# yolo_rtsp — RV1126B YOLOv8 目标检测 + ByteTrack 跟踪 + RTSP 推流

在 RV1126B IPC 开发板上运行的端侧 AI 应用：
`VI 采集 → YOLOv8 NPU 检测 → ByteTrack 多目标跟踪 → RGN 画框 → H264/H265 编码 → RTSP 推流`。

- 主码流 1920x1080 经 VENC 编码推流（RTSP `/live/0`）
- 推理流 640x360 送入 NPU 做 YOLOv8 检测
- 检测框 + 类别 + 置信度 + 跟踪 ID 通过 RGN 全屏 overlay 叠加到主码流
- 三段式异步流水线（预处理 / NPU 推理 / 后处理+跟踪）重叠执行，帧率约 25fps

## 功能特性

- YOLOv8 NPU 推理（rknn，运行时可配置类别数/阈值）
- ByteTrack 多目标跟踪，输出稳定跟踪 ID（`person 0.85 #3` 形式标签）
- 三段式流水线：预处理 / 推理 / 后处理+跟踪 三线程重叠
- RGN 硬件 overlay 画框 + 文字（内置 5x7 点阵字体）
- OSD：日期时间 + 帧率 + 目标数
- RTSP 推流（H264/H265 可选），可开关
- 摄像头 / 本地文件两种输入源
- 配置文件驱动（模型路径、端口、阈值、OSD 等）

## 目录结构

```
yolo_rtsp_app/
├── Makefile                      # 交叉编译 (source env 后 make)
├── include/
│   ├── yolov8.h                  # YOLO 推理公共接口
│   ├── postprocess.h             # 后处理/检测结果结构 (含 track_id)
│   ├── image_utils.h, common.h, file_utils.h
│   ├── bytetrack/                # ByteTrack 核心 (BYTETracker/STrack/kalmanFilter/lapjv)
│   └── opencv2/opencv.hpp        # 极简 OpenCV 兼容头 (见"依赖说明")
├── src/
│   ├── yolo_rtsp.cc              # 主程序: 采集/流水线/跟踪/RGN/推流
│   ├── yolo/                     # yolov8.cc, postprocess.cc
│   ├── bytetrack/                # ByteTrack 实现 (5 个 .cpp)
│   └── font5x7.h                 # 内置点阵字体
├── labels/                       # COCO 类别标签
└── lib/                          # 预编译运行库 (来自 SDK, 不入库)
```

## 依赖说明（重要）

本项目**没有引入完整 OpenCV 库**。ByteTrack 源码仅用到 `cv::Rect_<float>` 与 `cv::Scalar`
两个类型，因此提供了一份**极简 OpenCV 兼容头**：

- `include/opencv2/opencv.hpp` —— 只实现了 ByteTrack 需要的两个类型 + 常用 STL 头，
  无需在板端/交叉工具链中安装 OpenCV。
- 真正的第三方依赖是 **Eigen**（ByteTrack 卡尔曼滤波用）。Eigen 是纯头文件库，
  仅需在**编译服务器**上安装头文件，产物不依赖 Eigen 运行库。

> 注意：为规避 RV1126B (ARM NEON) 下 Eigen 的内存对齐断言崩溃，`include/bytetrack/dataType.h`
> 顶部已定义 `EIGEN_DONT_VECTORIZE` 与 `EIGEN_MAX_ALIGN_BYTES 0`。

## 编译

在编译服务器（Ubuntu，装有交叉工具链）上执行：

```sh
# 1) 安装 Eigen 头文件 (仅编译期需要)
sudo apt-get install -y libeigen3-dev

# 2) 载入 RV1126B 交叉编译环境
source /root/sdk/env-rv1126b.sh

# 3) 编译
cd yolo_rtsp_app
make yolo_rtsp        # 只编主程序
make                  # 或编全部 (stream_demo / yolo_img_test / yolo_rtsp)
```

产出 `yolo_rtsp`（ARM 可执行文件，约 17MB）。

### 离线单图测试工具

工程还提供两个**离线单图调试/验证**工具（不依赖摄像头/RTSP，用于在板端或服务器上直接对图片跑模型）：

| 可执行文件 | 用途 | 构建 |
|---|---|---|
| `yolo_img_test` | 单张图片 YOLOv8 检测 | `make yolo_img_test` |
| `car_img_test` | 单张图片 车牌/车辆识别（检测+车牌+车色） | `make car_img_test` |

> `make` / `make all` 会一并编译以上两个工具及主程序、web 守护进程。

#### yolo_img_test —— 单图 YOLOv8 检测

在板端（或安装了 SDK 运行库的环境）执行：

```sh
./yolo_img_test <model.rknn> <image> [labels.txt] [out.png] [iters] \
                [roi_x roi_y roi_w roi_h]
```

| 参数 | 说明 |
|---|---|
| `model.rknn` | YOLOv8 检测模型路径 |
| `image` | 输入图片路径 |
| `labels.txt` | 类别标签文件（可选，每行一个类别名） |
| `out.png` | 画框结果保存路径（默认 `out.png`） |
| `iters` | 推理次数（>1 时输出平均耗时，用于测速，默认 1） |
| `roi_x roi_y roi_w roi_h` | 主检测 ROI，0-1 归一化（可选，省略或全 0 = 全图检测） |

- 从原图裁剪 ROI → 缩放到模型尺寸 → 检测 → 反偏移回全图坐标，与主程序 VPSS
  硬件"裁剪+缩放"管线行为一致，便于离线调试 `main_roi` 检测效果。
- 输出打印检测到的目标（类别/置信度/框），并把检测框 + ROI 虚线框画到 `out.png`。

#### car_img_test —— 单图车牌/车辆识别

只链 `libcar.a`（纯识别库，不依赖 VPSS），输入一张图即可完整跑一遍
"车牌检测 → 车牌号/车牌颜色识别 → 车身颜色识别"管线：

```sh
./car_img_test <detect.rknn> <plate_model.rknn> <car_color.rknn> <image> \
               [plate_type] [roi_x roi_y roi_w roi_h] [out.jpg]
```

| 参数 | 说明 |
|---|---|
| `detect.rknn` | 车牌检测模型（检出车牌/车辆/两轮车三类） |
| `plate_model.rknn` | 车牌识别模型（`plate_rec_color*.rknn` 或 `lprnet.rknn`） |
| `car_color.rknn` | 车身颜色识别模型（`car_recognition.rknn`） |
| `image` | 输入图片路径 |
| `plate_type` | `0`=plate_rec(REC，默认)，`1`=LPRNet |
| `roi_x roi_y roi_w roi_h` | 识别 ROI，0-1 归一化（可选，省略或全 0 = 全图） |
| `out.jpg` | 画框结果保存路径（默认 `out_car.jpg`） |

- 打印每个目标的类型（Car / SinglePlate / DoublePlate）、置信度、车牌号、
  车牌颜色、车身颜色。
- 检测框 + ROI 虚线框 + 标签绘制后保存为 `out.jpg`。
- 若只关心单张车牌识别，给一张只含单个车牌的图即可当作"单张车牌识别"使用。

Makefile 关键点：
- 编译器 `arm-rockchip1240-linux-gnueabihf-g++`
- 需要 SDK 头文件路径（rockit/rkaiq/rknpu2）与预编译库（librtsp.a 等）
- Eigen 头文件路径 `-I/usr/include/eigen3`
- 链接 `-lrknnrt -lrockit -lrkaiq ...`

## 部署与运行

将 `yolo_rtsp` 放到板端 `/userdata/yolo/bin/` 并启动：

```sh
chmod +x /userdata/yolo/bin/yolo_rtsp
sh /userdata/yolo/run_yolo_rtsp.sh          # 自动读取 /userdata/yolo/yolo.conf
```

`run_yolo_rtsp.sh` 会先停止 rkipc（独占摄像头/554），启动 yolo_rtsp 及日志轮转守护。
退出后需手动恢复摄像头系统：`rkipc -a /oem/usr/share/iqfiles &`。

### 配置（yolo.conf）

```
model=/userdata/yolo/yolov8.rknn     # 模型路径
aiq=/oem/usr/share/iqfiles           # IQ 文件目录
width=1920 height=1080               # 主码流分辨率
codec=h264                           # h264 / h265
port=554                             # RTSP 端口
box_conf=0.25                        # 检测置信度阈值
nms_thresh=0.45                      # NMS 阈值
track_fps=30 track_buffer=30         # ByteTrack 帧率/缓冲参数
osd_scale=2                          # OSD 字体放大
show_osd=1 show_fps=1                # OSD 开关
push_stream=1                        # RTSP 推流开关
```

## 架构：三段式异步流水线

```
preprocess_thread  →  infer_thread  →  post_thread
 VI取帧/NV12转RGB/      inputs_set+      yolov8_postprocess (检测)
 letterbox 640x640      rknn_run(NPU)       ↓
                        outputs_get      byteTrack_update   (跟踪, 紧跟检测之后)
                                             ↓
                                          rgn_update (画框) / 统计 / 推流
```

- 3 个槽位 + 3 条队列（空闲/待推理/待后处理）连接三个线程，帧率上限 ≈ `rknn_run` ≈ 25fps。
- **检测与跟踪是顺序执行**的：`byteTrack_update` 在同一后处理线程内紧跟在
  `yolov8_postprocess` 之后，用当帧检测框更新跟踪器，再按 IoU 把跟踪 ID 回写回检测框。
- 跟踪 ID 叠加规则：标签显示为 `class 置信度 #ID`（如 `person 0.85 #3`），
  未被跟踪匹配的框 ID 为 `-1`。

## ByteTrack 集成说明

- 来源：ByteTrack 官方 C++ 版（TensorRT demo）中的纯跟踪核心，
  剥离了 TensorRT/完整 OpenCV 依赖，仅保留：
  `BYTETracker / STrack / kalmanFilter / lapjv / utils`。
- 依赖仅 Eigen + 极简 OpenCV 兼容头。
- 集成点：`src/yolo_rtsp.cc` 的 `byteTrack_update()`，构造 `Object` → `g_tracker.update()`
  → 按 IoU 匹配 `STrack.tlwh` 回写 `od.results[].track_id`。

## Git

仓库远端：`http://qingxia@10.10.31.201:12221/r/~qingxia/1126_project.git`
