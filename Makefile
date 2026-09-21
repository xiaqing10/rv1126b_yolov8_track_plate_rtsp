# =====================================================================
# RV1126B YOLO 检测 + RTSP 推流 + 车牌/车辆识别  独立工程 Makefile
# 使用: source /root/sdk/env-rv1126b.sh && make
# =====================================================================
P        := $(shell pwd)
SDK      := /root/sdk/rv1126b_linux_ipc_xiaoyu
CAR      := $(P)/src/car
CROSS    := arm-rockchip1240-linux-gnueabihf-
CC       := $(CROSS)gcc
CXX      := $(CROSS)g++

# ---- armhf OpenCV 3.4.5 (必须置于 $(P)/include 之前以屏蔽 shim) ----
OPENCV_DIR := $(SDK)/media/rknn-llm-mk/rknn-llm/examples/multimodal_model_demo/deploy/3rdparty/opencv/opencv-linux-armhf
OPENCV_INC := -I$(OPENCV_DIR)/include -I$(OPENCV_DIR)/include/opencv2
OPENCV_LIB := $(OPENCV_DIR)/lib
OPENCV_3RD := $(OPENCV_DIR)/share/OpenCV/3rdparty/lib

# ---- rockit / mpi 头 ----
RK_MPI_INC := $(SDK)/media/rockit/rockit/mpi/sdk/include
RK_DEFINES := $(SDK)/media/rockit/rockit/lib/arm/rv1126b
# ---- rkaiq 头 ----
RKAIQ      := $(SDK)/media/isp/camera_engine_rkaiq/rkaiq
RKAIQ_INC  := -I$(RKAIQ)/include -I$(RKAIQ)/uAPI2 -I$(RKAIQ)/include/uAPI2 \
              -I$(RKAIQ)/include/common -I$(RKAIQ)/include/xcore \
              -I$(RKAIQ)/include/algos -I$(RKAIQ)/include/iq_parser \
              -I$(RKAIQ)/include/iq_parser_v2
# ---- rknn / rtsp 头 ----
RKNPU2     := $(SDK)/project/app/testdemo/yoloworld_demo/3rdparty/rknpu2
RTSP_INC   := $(SDK)/media/common_algorithm/common_algorithm/misc/include

# 注意: OpenCV 头必须放在 $(P)/include 之前 (屏蔽 include/opencv2 shim)
INCS := $(OPENCV_INC) -I$(CAR) -I$(P)/include -I$(P)/include/deepsort -I$(P)/include/bytetrack -I$(P)/citradar_ab02 -I/usr/include/eigen3 \
        -I$(RK_MPI_INC) -I$(RK_DEFINES) \
        -I$(RKNPU2)/include -I$(RTSP_INC) $(RKAIQ_INC)

# DeepSORT 崩溃根因 (Eigen 对齐分配器):
# 32 位 armv7 栈/结构体默认只 8 字节对齐, 而 Eigen 固定尺寸矩阵
# (KAL_HCOVA 4x4、KAL_COVA 8x8=256B) 在 NEON 下被强制 16 字节对齐,
# 走 handmade_aligned_malloc (把原始基址存于 data-1), free 时读 *(data-1)。
# 与 -O2 的 move/惰性表达式结合, *(data-1) 书签被写坏 -> free(垃圾) 段错误
# (有目标即崩的根因)。
# 修复: EIGEN_DONT_ALIGN 强制 EIGEN_MAX_ALIGN_BYTES=0 且
# EIGEN_MAX_STATIC_ALIGN_BYTES=0, 让固定尺寸与动态矩阵全部走 plain malloc/free,
# 彻底绕开 handmade 对齐分配器。注意: 勿用 EIGEN_MALLOC_ALREADY_ALIGNED=1,
# 板端 glibc malloc 只 8 字节对齐, 会触发 aligned_malloc 断言。
CFLAGS := -g -Wall -O2 -fno-omit-frame-pointer -DRV1126B -DUAPI2 -DRKPLATFORM=ON -DARCH64=OFF \
          -DEIGEN_DONT_ALIGN \
          -DEIGEN_MAX_ALIGN_BYTES=0 \
          -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -fopenmp $(INCS)
CXXFLAGS := $(CFLAGS) -std=c++14

LIBDIR := -L$(P)/lib -L$(OPENCV_LIB) -L$(OPENCV_3RD)

# OpenCV 静态库 (仅需 core+imgproc, 附 3rdparty 依赖以满足符号)
OPENCV_LIBS := -lopencv_imgcodecs -lopencv_imgproc -lopencv_core \
               -lIlmImf -llibjasper -llibjpeg-turbo -llibpng -llibtiff \
               -llibwebp -lquirc -lzlib -ltegra_hal

LIBS := -Wl,--start-group \
        $(P)/lib/librtsp.a \
        -lrockit -lrockchip_mpp -lrkaiq -lrga -lrksysutils -lrknnrt \
        -lasound -ldrm \
        $(OPENCV_LIBS) \
        -lpthread -lm -lstdc++ -ldl -fopenmp \
        -Wl,--end-group
LDFLAGS := $(LIBDIR) $(LIBS) -Wl,-rpath,/oem/usr/lib:/oem/lib -Wl,--gc-sections -Wl,--as-needed

# ---- YOLO 推理公共源码 ----
YOLO_SRC := src/yolo/yolov8.cc src/yolo/postprocess.cc \
            src/image_utils.cc src/file_utils.cc
# ---- ByteTrack 跟踪库 ----
TRACK_SRC := src/bytetrack/BYTETracker.cpp src/bytetrack/STrack.cpp \
             src/bytetrack/kalmanFilter.cpp src/bytetrack/lapjv.cpp \
             src/bytetrack/utils.cpp
# ---- DeepSORT 跟踪库 (ReID: Kalman + 匈牙利级联匹配; 特征由后台 worker 异步推理) ----
DEEPSORT_SRC := src/deepsort/kalmanfilter.cpp src/deepsort/track.cpp \
                src/deepsort/tracker.cpp src/deepsort/nn_matching.cpp \
                src/deepsort/linear_assignment.cpp src/deepsort/hungarianoper.cpp \
                src/deepsort/reid_rknn.cpp src/deepsort/deepsort_integ.cpp
# ---- 最小基线: 读 h264/h265 裸流 -> VDEC -> VPSS -> VENC -> RTSP (不依赖 OpenCV/AI) ----
# 仅需 rockit + librtsp, 精简链接避免拖入 OpenCV 依赖
TEST_RTSP_LIBS := -Wl,--start-group $(P)/lib/librtsp.a -lrockit -lrockchip_mpp \
                  -lrga -lrksysutils -lasound -ldrm -lpthread -lm -ldl -Wl,--end-group
test_vdec_h264_rtsp: src/test_vdec_h264_rtsp.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBDIR) $(TEST_RTSP_LIBS) -Wl,-rpath,/oem/usr/lib:/oem/lib -Wl,--gc-sections -Wl,--as-needed
	@echo "build test_vdec_h264_rtsp OK"

# 官方 sample_vdec_vpss_vo_stresstest 改造: VDEC->VPSS->VENC 绑定 + 官方紧循环喂流(无sleep) + RTSP
test_vdec_vpss_venc_rtsp: src/test_vdec_vpss_venc_rtsp.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBDIR) $(TEST_RTSP_LIBS) -Wl,-rpath,/oem/usr/lib:/oem/lib -Wl,--gc-sections -Wl,--as-needed
	@echo "build test_vdec_vpss_venc_rtsp OK"

# ---- 车牌/车辆识别管线 (car_lib 纯识别库, 不依赖 VPSS) ----
CAR_LIB_SRC := $(CAR)/detect.cc $(CAR)/plate_rec.cc $(CAR)/car_color.cc \
               $(CAR)/pipeline.cc $(CAR)/lprnet.cc
# ---- 识别集成: worker 线程(拉帧) + 主线程消费/裁决 ----
CAR_INTEG_SRC := src/car_worker.cc src/car_integ.cc
# ---- TCP + NDJSON 输出 (standalone, POSIX socket only) ----
TCP_SRC := src/tcp_output.cc
# ---- CitRadar AB02 协议发送 (客户端外连, 二进制) ----
AB02_SRC := citradar_ab02/citradar_ab02.cpp

# ---- Web control daemon (standalone, pthread+dl only, no SDK libs) ----
web/yolo_web: web/yolo_web.c
	$(CC) -O2 -Wall -o web/yolo_web web/yolo_web.c -pthread -ldl
	@echo "build web/yolo_web OK"

default: all

# 里程碑1: 采集 -> 编码 -> RTSP (官方样例, 验证骨架)
stream_demo: src/simple_vi_bind_venc_rtsp.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)
	@echo "build stream_demo OK"

# 里程碑2: 静态图 YOLOv8 推理 (验证模型在 NPU 上可跑; 支持可选主线程 ROI 0-1 离线调试检测)
yolo_img_test: src/yolo_img_test.cc $(YOLO_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)
	@echo "build yolo_img_test OK"

# ---- car_lib: 纯识别静态库 (不依赖 VPSS/rockit, 可独立单图测试) ----
lib/libcar.a: $(CAR_LIB_SRC) $(CAR)/car_recognition.h
	@mkdir -p lib
	$(CXX) $(CXXFLAGS) -c $(CAR_LIB_SRC)
	$(AR) rcs $@ detect.o plate_rec.o car_color.o pipeline.o lprnet.o
	@rm -f detect.o plate_rec.o car_color.o pipeline.o lprnet.o
	@echo "build lib/libcar.a OK"

# 单图测试: 输入一张图, 验证能否检出车牌 (只链 car_lib)
car_img_test: src/car_img_test.cc lib/libcar.a
	$(CXX) $(CXXFLAGS) $< -o $@ lib/libcar.a $(LIBDIR) $(LIBS)
	@echo "build car_img_test OK"

# 里程碑3: 完整管线 VI->YOLO->RGN画框->RTSP (+车牌/车辆识别 +DeepSORT)
yolo_rtsp: src/yolo_rtsp.cc $(YOLO_SRC) $(TRACK_SRC) $(DEEPSORT_SRC) $(CAR_INTEG_SRC) $(TCP_SRC) $(AB02_SRC) lib/libcar.a
	$(CXX) $(CXXFLAGS) -DCITRADAR_AB02_NO_MAIN $^ -o $@ $(LDFLAGS)
	@echo "build yolo_rtsp OK"

all: stream_demo yolo_img_test car_img_test yolo_rtsp web/yolo_web

clean:
	rm -f stream_demo yolo_img_test car_img_test yolo_rtsp web/yolo_web
	# 仅删除本工程构建出的 libcar.a; 保留 lib/ 下 SDK 预编译库 (librtsp.a 及 *.so)
	rm -f lib/libcar.a
