// =====================================================================
// RV1126B: VI 采集 -> YOLOv8 NPU 检测 -> RGN 画框 -> H264/H265 -> RTSP
//
//   VI chn0 (主码流 1920x1080, 16:9) --bind--> VENC0 --> RTSP /live/0
//   VI chn1 (推理流 长边640 等比, 跟随主 ROI 宽高比) --GetChnFrame--> NV12->RGB -> YOLO
//   RGN 全屏 ARGB8888 overlay attach 到 VENC0, 每次检测后 SetBitMap 画框
//
//   用法:
//   ./yolo_rtsp -m /userdata/yolov8.rknn -a /oem/usr/share/iqfiles \
//               -w 1920 -h 1080 -e h264 -p 554
// =====================================================================

// 版本号: 改动功能/修复 bug 时递增, 启动时打印到日志便于排查
#ifndef APP_VERSION
#define APP_VERSION "1.0.0"
#endif
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "rtsp_demo.h"
#include "font5x7.h"
#include "font_plate.h"

#include <rk_aiq_user_api2_camgroup.h>
#include <rk_aiq_user_api2_imgproc.h>
#include <rk_aiq_user_api2_sysctl.h>

#include "rk_debug.h"
#include "rk_defines.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_cal.h"
#include "rk_mpi_mmz.h"
#include "rk_mpi_rgn.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_vdec.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"
#include "rk_mpi_vpss.h"

#include "yolov8.h"
#include "image_utils.h"
#include "bytetrack/BYTETracker.h"
#include "car_integ.h"
#include "tcp_output.h"
#include "deepsort/deepsort_integ.h"

#define ALIGN16(x) ((((x) + 15) / 16) * 16)
#define AIQ_CTX_MAX 8

static rk_aiq_sys_ctx_t *g_aiq_ctx[AIQ_CTX_MAX];
static rk_aiq_working_mode_t g_wdr_mode[AIQ_CTX_MAX];

static volatile bool g_quit = false;
static int g_debug = 0;      // 1=打印所有调试信息; 0=只打印帧率与检测到的车牌
static rtsp_demo_handle g_rtsp = NULL;
static rtsp_session_handle g_rtsp_sess = NULL;

// 视频参数 (主码流 / AI 流均为 16:9, 坐标线性映射)
static RK_U32 g_main_w = 1920;
static RK_U32 g_main_h = 1080;
static RK_U32 g_ai_w   = 640;   // 整幅-AI 坐标系基准宽 (整幅源图映射), 供反偏移/OSD/车牌关联
static RK_U32 g_ai_h   = 360;
#define AI_CHN_REF 640          // chn1 输出长边基准 (与 g_ai_w 默认一致; 保证反偏移系数简化)
static RK_U32 g_chn_w = 640;    // chn1 实际输出宽 (动态: 长边=640, 短边按裁剪区域宽高比等比)
static RK_U32 g_chn_h = 360;    // chn1 实际输出高
static RK_U32 g_vi_w   = 3840;   // 摄像头 VI 源分辨率 (传感器原生 4K)
static RK_U32 g_vi_h   = 2160;
static RK_U32 g_bitrate = 4096;                 // Kbps
static RK_CODEC_ID_E g_codec = RK_VIDEO_ID_AVC;
static int g_rtsp_port = 8554;
static char g_rtsp_path[64] = "/live/0";
static char g_iq_dir[256]   = "/oem/usr/share/iqfiles";
static int  g_show_osd = 1;     // 显示日期时间
static int  g_show_fps = 1;     // 显示帧率/目标数
static int  g_osd_scale = 2;    // OSD 字体放大倍数 (5x7*scale), 默认 2
static int  g_osd_plate_scale = 1;  // 车牌号文字放大倍数 (独立于 osd_scale), 默认 1
static int  g_show_plate = 0;       // 显示车牌内容开关 (1=在轨迹框+独立车牌框显示车牌, 0=隐藏)
static int  g_have_plate_text = 0;  // 当前帧是否绘制了车牌号文字(用于只在有车牌时存图)
static RK_U32 g_osd_color = 0xFFFFFFFFu;  // OSD 文字颜色 (ARGB, 默认白)
static int  g_push_stream = 1;  // 是否开启 RTSP 推流 (1=开 0=关, 关闭仅用于排查/省带宽)
static double g_fps = 0.0;
static int  g_print_interval_s = 5;   // 运行统计日志 (fps/检测/跟踪/车牌耗时) 打印间隔 (秒)

// 主线程检测 ROI (0-1 归一化, 相对源帧/整幅图; 全 0 = 整幅不裁剪)
// VPSS chn1 硬件裁剪该区域并缩放, 目标放大提升检测; 检测结果在 post_thread 反偏移回整幅坐标。
static float g_main_roi_x = 0.0f, g_main_roi_y = 0.0f;
static float g_main_roi_w = 0.0f, g_main_roi_h = 0.0f;
// 裁剪生效时 chn1 输出即 ROI 区域; 反偏移换算用 (AI 尺寸 + 归一化 ROI)
static int g_main_roi_enable = 0;   // 1=主线程 ROI 裁剪生效
static float g_main_roi_ai_x0 = 0.0f, g_main_roi_ai_y0 = 0.0f;  // ROI 左上角 (AI 坐标)
static float g_main_roi_ai_sx = 1.0f, g_main_roi_ai_sy = 1.0f;  // 缩放系数 (归一化 ROI 宽高)

// ---- TCP + NDJSON 输出 (frame/detection/plate_adopted) ----
static tcp_output_cfg_t g_tcp_cfg;   // enable/port 由 config 填充
static long g_tcp_frame_seq = 0;     // frame 消息自增序号

// ---- 输入源选择 ----
static int  g_file_mode = 0;            // 0=摄像头 1=本地视频文件
static char g_video_file[256] = "";     // 文件模式: .h264/.h265 裸流路径
static RK_U32 g_dec_w = 1920;           // 文件解码原始宽
static RK_U32 g_dec_h = 1080;
static RK_CODEC_ID_E g_dec_codec = RK_VIDEO_ID_AVC;  // 文件解码编码类型
static int  g_file_fps = 25;            // 文件回放推帧帧率

// 媒体组件创建状态(用于失败/退出时安全释放)
static int g_inited_sys = 0, g_inited_vi = 0, g_inited_vdec = 0;
static int g_inited_vpss = 0, g_inited_venc = 0, g_inited_rgn = 0;
static int g_bound_main = 0;   // 主码流->VENC 已绑定
static int g_bound_vdec = 0;  // VDEC->VPSS 已绑定
static MB_POOL g_vdec_pool = MB_INVALID_POOLID;  // VDEC 手动模式帧缓冲池
// 模型
static char g_model_path[256] = "/userdata/yolo/yolov8.rknn";
static rknn_app_context_t g_yolo;
static pthread_mutex_t g_od_lock = PTHREAD_MUTEX_INITIALIZER;
static object_detect_result_list g_od;

// 跟踪器配置 (tracker=bytetrack|deepsort)
static int   g_tracker_type = 0;      // 0=ByteTrack 1=DeepSORT
static char  g_reid_model[256] = "/userdata/reid/osnet_x1_0_imagenet.rknn";
static int   g_reid_dim = 512;
static int   g_reid_w = 128, g_reid_h = 256;
static int   g_reid_interval = 3;   // DeepSORT: 每隔 N 帧才对全部目标做一次 ReID 推理 (省 NPU, 提帧率)

// ByteTrack 跟踪器 (只在后处理线程使用, 无需加锁)
static BYTETracker g_tracker(30, 30);

// RGN 全屏 overlay
static RGN_HANDLE g_rgn = 0;
static RK_U32 *g_overlay = NULL;
static RK_U32 g_ov_w = 0, g_ov_h = 0;

// [PLATE] 日志限速: 每个 track_id 的车牌号至多每 PRINT_GAP_MS 打印一次, 避免回填后刷屏
#define PLATE_PRINT_GAP_MS 1000
#define PLATE_PRINT_MAX 64
typedef struct { int track_id; char num[32]; long long last_ms; } plate_print_t;
static plate_print_t g_plate_print[PLATE_PRINT_MAX];
static int plate_print_gate(int track_id, const char *num)
{
    if (track_id < 0 || track_id >= PLATE_PRINT_MAX) return 0;
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    long long now = (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
    plate_print_t *e = &g_plate_print[track_id];
    if (e->track_id != track_id || strcmp(e->num, num) != 0 ||
        now - e->last_ms >= PLATE_PRINT_GAP_MS) {
        e->track_id = track_id; snprintf(e->num, sizeof(e->num), "%s", num); e->last_ms = now;
        return 1;
    }
    return 0;
}

static void on_signal(int sig) { (void)sig; g_quit = true; }

// ---------------- 配置文件解析 (key = value, '#' 注释) ----------------
static char *cfg_trim(char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    if (*s == 0) return s;
    char *e = s + strlen(s) - 1;
    while (e > s && isspace((unsigned char)*e)) { *e = 0; e--; }
    return s;
}

// 解析 OSD 颜色配置: 支持颜色名或 ARGB 十六进制(如 0xFFFF00FF / FF00FF00)
// 注意 RK overlay 内存为 BGRA 字序, 故 R=G 通道互换(见下方 COL_* 宏注释)
 static RK_U32 cfg_parse_color(const char *v) {
     // 返回值按 BGRA 字序(A 在最低字节), alpha 须为 0xFF 才不透明显示
     if (!strcmp(v, "white"))      return 0xFFFFFFFFu;
     if (!strcmp(v, "black"))      return 0x000000FFu;
     if (!strcmp(v, "red"))        return 0x0000FFFFu;
     if (!strcmp(v, "green"))      return 0x00FF00FFu;
     if (!strcmp(v, "blue"))       return 0xFF0000FFu;
     if (!strcmp(v, "yellow"))     return 0x00FFFFFFu;
     if (!strcmp(v, "cyan"))       return 0xFFFF00FFu;
     if (!strcmp(v, "magenta"))    return 0x0000FFFFu;
    // 十六进制: 0xRRGGBBAA 或 RRGGBBAA (按 BGRA 字序解析, 直接给值)
    unsigned long uv = strtoul(v, NULL, 0);
    if (uv) return (RK_U32)uv;
    return 0xFFFFFFFFu;  // 默认白
}

static void cfg_set(const char *k, const char *v) {
    if (!strcmp(k, "model"))       snprintf(g_model_path, sizeof(g_model_path), "%s", v);
    else if (!strcmp(k, "iq_dir")) snprintf(g_iq_dir, sizeof(g_iq_dir), "%s", v);
    else if (!strcmp(k, "labels")) snprintf(g_label_path, sizeof(g_label_path), "%s", v);
    else if (!strcmp(k, "width"))  g_main_w = atoi(v);
    else if (!strcmp(k, "height")) g_main_h = atoi(v);
    else if (!strcmp(k, "vi_w"))   g_vi_w = atoi(v);
    else if (!strcmp(k, "vi_h"))   g_vi_h = atoi(v);
    else if (!strcmp(k, "ai_w"))   g_ai_w = atoi(v);
    else if (!strcmp(k, "bitrate")) g_bitrate = atoi(v);
    else if (!strcmp(k, "port"))   g_rtsp_port = atoi(v);
    else if (!strcmp(k, "path"))   snprintf(g_rtsp_path, sizeof(g_rtsp_path), "%s", v);
    else if (!strcmp(k, "codec"))  g_codec = (!strcmp(v, "h265")) ? RK_VIDEO_ID_HEVC : RK_VIDEO_ID_AVC;
    else if (!strcmp(k, "conf_thresh")) g_box_conf_thresh = (float)atof(v);
    else if (!strcmp(k, "nms_thresh"))  g_nms_thresh = (float)atof(v);
    else if (!strcmp(k, "class_num"))   g_obj_class_num = atoi(v);   // 0=自动
    else if (!strcmp(k, "show_osd"))    g_show_osd = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "show_fps"))    g_show_fps = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "osd_scale"))   g_osd_scale = atoi(v) > 0 ? atoi(v) : 2;
    else if (!strcmp(k, "osd_plate_scale")) g_osd_plate_scale = atoi(v) > 0 ? atoi(v) : 1;
    else if (!strcmp(k, "osd_color"))   g_osd_color = cfg_parse_color(v);
    else if (!strcmp(k, "show_plate"))  g_show_plate = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "push_stream")) g_push_stream = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "debug"))       g_debug = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "video"))       { snprintf(g_video_file, sizeof(g_video_file), "%s", v); g_file_mode = 1; }
    else if (!strcmp(k, "file_fps"))    g_file_fps = atoi(v) ? atoi(v) : 25;
    else if (!strcmp(k, "dec_w"))       g_dec_w = atoi(v);
    else if (!strcmp(k, "dec_h"))       g_dec_h = atoi(v);
    else if (!strcmp(k, "vcodec"))      g_dec_codec = (!strcmp(v, "h265")) ? RK_VIDEO_ID_HEVC : RK_VIDEO_ID_AVC;
    // ---- 车牌/车辆识别集成 ----
    else if (!strcmp(k, "car_enable"))        g_car_cfg.enable = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "car_src_mode"))      g_car_cfg.src_mode = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "car_roi_x"))         g_car_cfg.roi_nx = (float)atof(v);
    else if (!strcmp(k, "car_roi_y"))         g_car_cfg.roi_ny = (float)atof(v);
    else if (!strcmp(k, "car_roi_w"))         g_car_cfg.roi_nw = (float)atof(v);
    else if (!strcmp(k, "car_roi_h"))         g_car_cfg.roi_nh = (float)atof(v);
    else if (!strcmp(k, "car_chn"))           g_car_cfg.chn = atoi(v);
    else if (!strcmp(k, "car_interval_ms"))   g_car_cfg.interval_ms = atoi(v);
    else if (!strcmp(k, "car_conf_thresh"))   g_car_cfg.detect_conf_thresh = atof(v);
    else if (!strcmp(k, "car_nms_thresh"))    g_car_cfg.detect_nms_thresh  = atof(v);
    else if (!strcmp(k, "car_plate_nms_thresh")) g_car_cfg.plate_nms_thresh = atof(v);
    else if (!strcmp(k, "car_plate_type"))    g_car_cfg.plate_type = (!strcmp(v, "lprnet") || atoi(v) == 1) ? 1 : 0;
    else if (!strcmp(k, "car_detect_model"))  snprintf(g_car_cfg.detect_model, sizeof(g_car_cfg.detect_model), "%s", v);
    else if (!strcmp(k, "car_plate_model"))   snprintf(g_car_cfg.plate_model, sizeof(g_car_cfg.plate_model), "%s", v);
    else if (!strcmp(k, "car_car_model"))     snprintf(g_car_cfg.car_model, sizeof(g_car_cfg.car_model), "%s", v);
    else if (!strcmp(k, "car_plate_vote"))    g_car_cfg.plate_vote_thresh = atoi(v);
    else if (!strcmp(k, "car_color_vote"))    g_car_cfg.car_color_vote_thresh = atoi(v);
    else if (!strcmp(k, "car_track_age"))     g_car_cfg.max_track_age_frames = atoi(v);
    else if (!strcmp(k, "car_match_iou"))     g_car_cfg.match_iou_thresh = atof(v);
    // ---- 车牌检测图片保存 ----
    else if (!strcmp(k, "save_plate_image"))  g_car_cfg.save_plate_image = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "plate_image_path"))  snprintf(g_car_cfg.save_dir, sizeof(g_car_cfg.save_dir), "%s", v);
    else if (!strcmp(k, "plate_image_interval")) g_car_cfg.save_interval_ms = atoi(v);
    // ---- 主线程检测 ROI (0-1 归一化, 相对整幅; VPSS chn1 硬件裁剪+缩放) ----
    else if (!strcmp(k, "main_roi_x")) g_main_roi_x = (float)atof(v);
    else if (!strcmp(k, "main_roi_y")) g_main_roi_y = (float)atof(v);
    else if (!strcmp(k, "main_roi_w")) g_main_roi_w = (float)atof(v);
    else if (!strcmp(k, "main_roi_h")) g_main_roi_h = (float)atof(v);
    // ---- TCP + NDJSON 输出 ----
    else if (!strcmp(k, "tcp_enable"))    g_tcp_cfg.enable = atoi(v) ? 1 : 0;
    else if (!strcmp(k, "tcp_port"))      g_tcp_cfg.port = atoi(v);
    else if (!strcmp(k, "tcp_max_clients")) g_tcp_cfg.max_clients = atoi(v);
    // ---- 跟踪器 (bytetrack|deepsort) + ReID 模型 ----
    else if (!strcmp(k, "tracker"))   g_tracker_type = (!strcmp(v, "deepsort")) ? 1 : 0;
    else if (!strcmp(k, "reid_model")) snprintf(g_reid_model, sizeof(g_reid_model), "%s", v);
    else if (!strcmp(k, "reid_dim"))  g_reid_dim = atoi(v) > 0 ? atoi(v) : 512;
    else if (!strcmp(k, "reid_w"))    g_reid_w = atoi(v) > 0 ? atoi(v) : 128;
    else if (!strcmp(k, "reid_h"))    g_reid_h = atoi(v) > 0 ? atoi(v) : 256;
    else if (!strcmp(k, "reid_interval")) g_reid_interval = atoi(v) > 0 ? atoi(v) : 3;
    else if (!strcmp(k, "print_interval_s")) g_print_interval_s = atoi(v) > 0 ? atoi(v) : 5;
    else printf("config: unknown key '%s'\n", k);
}

static int load_config(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) { printf("config not found: %s (use defaults)\n", path); return -1; }
    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        char *p = cfg_trim(line);
        if (*p == 0 || *p == '#' || *p == ';') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = 0;
        char *k = cfg_trim(p);
        char *v = cfg_trim(eq + 1);
        // 剥离值里的行内注释: 支持 '#' 或 ';' (引号内的暂不处理)
        char *hash = v;
        while (*hash) {
            if (*hash == '#' || *hash == ';') { *hash = 0; break; }
            hash++;
        }
        v = cfg_trim(v);
        cfg_set(k, v);
    }
    fclose(fp);
    printf("loaded config %s\n", path);
    return 0;
}

// ---------------- ISP (rkaiq) ----------------
static XCamReturn isp_sof_cb(rk_aiq_metas_t *m) { (void)m; return XCAM_RETURN_NO_ERROR; }
static XCamReturn isp_err_cb(rk_aiq_err_msg_t *e) {
    if (e && e->err_code == XCAM_RETURN_BYPASS) g_quit = true;
    return XCAM_RETURN_NO_ERROR;
}

static int isp_init(int cam_id, const char *iq_dir) {
    setlinebuf(stdout);
    if (!iq_dir) { g_aiq_ctx[cam_id] = NULL; return 0; }
    setenv("HDR_MODE", "0", 1);
    g_wdr_mode[cam_id] = RK_AIQ_WORKING_MODE_NORMAL;

    rk_aiq_static_info_t info;
    memset(&info, 0, sizeof(info));
    rk_aiq_uapi2_sysctl_enumStaticMetas(cam_id, &info);
    printf("ISP[%d] sensor=%s iq=%s\n", cam_id, info.sensor_info.sensor_name, iq_dir);

    g_aiq_ctx[cam_id] = rk_aiq_uapi2_sysctl_init(info.sensor_info.sensor_name, iq_dir,
                                                 isp_err_cb, isp_sof_cb);
    if (!g_aiq_ctx[cam_id]) { printf("rkaiq init fail\n"); return -1; }
    if (rk_aiq_uapi2_sysctl_prepare(g_aiq_ctx[cam_id], 0, 0, g_wdr_mode[cam_id]) != RK_SUCCESS) {
        printf("rkaiq prepare fail\n"); return -1;
    }
    if (rk_aiq_uapi2_sysctl_start(g_aiq_ctx[cam_id]) != RK_SUCCESS) {
        printf("rkaiq start fail\n"); return -1;
    }
#ifdef RV1126B
    if (g_aiq_ctx[cam_id] && rk_aiq_uapi2_judgeNeedRawPipeCtrl(g_aiq_ctx[cam_id]))
        RK_MPI_VI_CtrlRawStream(RK_FALSE);
    else
        RK_MPI_VI_CtrlRawStream(RK_TRUE);
#endif
    return 0;
}

static void isp_deinit(int cam_id) {
    if (g_aiq_ctx[cam_id]) {
        rk_aiq_uapi2_sysctl_stop(g_aiq_ctx[cam_id], false);
        rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx[cam_id]);
        g_aiq_ctx[cam_id] = NULL;
    }
}
// ---------------- VI ----------------
static int vi_dev_init(void) {
    int ret;
    VI_DEV_ATTR_S dev; VI_DEV_BIND_PIPE_S bind;
    memset(&dev, 0, sizeof(dev));
    memset(&bind, 0, sizeof(bind));
    ret = RK_MPI_VI_GetDevAttr(0, &dev);
    if (ret == RK_ERR_VI_NOT_CONFIG) {
        ret = RK_MPI_VI_SetDevAttr(0, &dev);
        if (ret) { printf("SetDevAttr %x\n", ret); return -1; }
    }
    ret = RK_MPI_VI_GetDevIsEnable(0);
    if (ret != RK_SUCCESS) {
        ret = RK_MPI_VI_EnableDev(0);
        if (ret) { printf("EnableDev %x\n", ret); return -1; }
        bind.u32Num = 1; bind.PipeId[0] = 0;
        ret = RK_MPI_VI_SetDevBindPipe(0, &bind);
        if (ret) { printf("SetDevBindPipe %x\n", ret); return -1; }
    }
    return 0;
}

static int vi_chn_init(int chn, int w, int h, int depth) {
    VI_CHN_ATTR_S attr;
    memset(&attr, 0, sizeof(attr));
    attr.stIspOpt.u32BufCount = 4;
    attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    attr.stSize.u32Width = w;
    attr.stSize.u32Height = h;
    attr.enPixelFormat = RK_FMT_YUV420SP;   // NV12
    attr.enCompressMode = COMPRESS_MODE_NONE;
    attr.u32Depth = depth;                    // 绑定通道=0; 取帧通道>=1
    int ret = RK_MPI_VI_SetChnAttr(0, chn, &attr);
    ret |= RK_MPI_VI_EnableChn(0, chn);
    if (ret) { printf("VI chn%d enable fail %x\n", chn, ret); return -1; }
    return 0;
}

// ---------------- VENC ----------------
static int venc_init(int chn, int w, int h) {
    VENC_CHN_ATTR_S attr;
    VENC_RECV_PIC_PARAM_S recv;
    memset(&attr, 0, sizeof(attr));
    attr.stVencAttr.enType = g_codec;
    attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
    attr.stVencAttr.u32PicWidth = w;
    attr.stVencAttr.u32PicHeight = h;
    attr.stVencAttr.u32VirWidth = w;
    attr.stVencAttr.u32VirHeight = h;
    attr.stVencAttr.u32StreamBufCnt = 2;
    attr.stVencAttr.u32BufSize = w * h * 3 / 2;
    attr.stVencAttr.enMirror = MIRROR_NONE;
    if (g_codec == RK_VIDEO_ID_AVC) {
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        attr.stRcAttr.stH264Cbr.u32BitRate = g_bitrate;
        attr.stRcAttr.stH264Cbr.u32Gop = 60;
        attr.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
    } else {
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        attr.stRcAttr.stH265Cbr.u32BitRate = g_bitrate;
        attr.stRcAttr.stH265Cbr.u32Gop = 60;
    }
    int ret = RK_MPI_VENC_CreateChn(chn, &attr);
    if (ret) { printf("VENC create fail %x\n", ret); return -1; }
    memset(&recv, 0, sizeof(recv));
    recv.s32RecvPicNum = -1;
    RK_MPI_VENC_StartRecvFrame(chn, &recv);
    return 0;
}

// ---------------- VPSS 分流 (摄像头与文件模式共用) ----------------
// 建 VPSS grp0, 输出:
//   chn0: 主码流 g_main_w x g_main_h (深度0, 绑定 VENC)
//   chn1: AI 流   长边640 等比(跟随主 ROI 宽高比, 深度2, preprocess 手动取帧)
//   chn2: 车牌    src_w x src_h       (深度2, car_integ 手动取帧, 源分辨率保细节)
static int vpss_pipeline_init(int src_w, int src_h) {
    VPSS_GRP_ATTR_S gattr;
    memset(&gattr, 0, sizeof(gattr));
    gattr.u32MaxW = src_w;
    gattr.u32MaxH = src_h;
    gattr.enPixelFormat = RK_FMT_YUV420SP;
    gattr.stFrameRate.s32SrcFrameRate = -1;
    gattr.stFrameRate.s32DstFrameRate = -1;
    gattr.enCompressMode = COMPRESS_MODE_NONE;
#ifdef RV1126B
    gattr.enVProcDev = VIDEO_PROC_DEV_VPSS;
    gattr.u32MaxQueue = 10;
#endif
    int ret = RK_MPI_VPSS_CreateGrp(0, &gattr);
    if (ret) { printf("VPSS create grp fail %x\n", ret); return -1; }

    VPSS_CHN_ATTR_S cattr;
    memset(&cattr, 0, sizeof(cattr));
    cattr.enChnMode = VPSS_CHN_MODE_USER;
    cattr.enDynamicRange = DYNAMIC_RANGE_SDR8;
    cattr.enPixelFormat = RK_FMT_YUV420SP;
    cattr.u32Width = g_main_w;
    cattr.u32Height = g_main_h;
    cattr.u32Depth = 0;
    cattr.u32FrameBufCnt = 8;
    cattr.stMaxSize.u32Width = src_w;
    cattr.stMaxSize.u32Height = src_h;
    cattr.stFrameRate.s32SrcFrameRate = 25;
    cattr.stFrameRate.s32DstFrameRate = 25;   // 限制主码流 25fps, 避免 VENC 超速编码 -> 每帧码率不足马赛克
    cattr.enCompressMode = COMPRESS_MODE_NONE;
    RK_MPI_VPSS_SetChnAttr(0, 0, &cattr);
    RK_MPI_VPSS_EnableChn(0, 0);

    // chn1 输出尺寸跟随裁剪区域宽高比 (固定长边=640, 短边等比), 经 letterbox 转模型输入:
    // 避免把 ROI 固定拉伸到 640x360 造成的非等比失真。长边基准 = AI_CHN_REF, 与整幅-AI
    // 基准 g_ai_w 一致, 使反偏移系数在横向 ROI 时简化为归一化 ROI 宽高 (见下)。
    // 裁剪区域宽高比: 有主 ROI 用 ROI 比例, 无 ROI 用源图比例。
    {
        float ar = (g_main_roi_w > 0.0f && g_main_roi_h > 0.0f)
                 ? (g_main_roi_h / g_main_roi_w)
                 : ((float)src_h / (float)src_w);
        if (ar >= 1.0f) {          // 竖向: 高为长边
            g_chn_h = AI_CHN_REF;
            g_chn_w = (RK_U32)(g_chn_h / ar + 0.5f);
        } else {                   // 横向/方形: 宽为长边
            g_chn_w = AI_CHN_REF;
            g_chn_h = (RK_U32)(g_chn_w * ar + 0.5f);
        }
        g_chn_w &= ~1;  g_chn_h &= ~1;   // VPSS NV12 需偶数对齐
        if (g_chn_w < 2) g_chn_w = 2;
        if (g_chn_h < 2) g_chn_h = 2;
    }

    memset(&cattr, 0, sizeof(cattr));
    cattr.enChnMode = VPSS_CHN_MODE_USER;
    cattr.enDynamicRange = DYNAMIC_RANGE_SDR8;
    cattr.enPixelFormat = RK_FMT_YUV420SP;
    cattr.u32Width = g_chn_w;
    cattr.u32Height = g_chn_h;
    cattr.u32Depth = 2;             // 手动 GetChnFrame 通道需要 depth>1
    cattr.u32FrameBufCnt = 8;
    cattr.stMaxSize.u32Width = src_w;
    cattr.stMaxSize.u32Height = src_h;
    cattr.stFrameRate.s32SrcFrameRate = -1;
    cattr.stFrameRate.s32DstFrameRate = -1;
    cattr.enCompressMode = COMPRESS_MODE_NONE;
    RK_MPI_VPSS_SetChnAttr(0, 1, &cattr);
    RK_MPI_VPSS_EnableChn(0, 1);

    // 主线程检测 ROI: VPSS chn1 硬件"源图裁剪 -> 缩放输出", 放大 ROI 目标提升检测。
    // 裁剪用 VPSS_CROP_ABS_COOR(绝对像素坐标, 相对源图 src_w x src_h), 由 0-1 归一化换算,
    // 避免 RATIO_COOR 比例刻度在不同 SDK 版本不一致导致的裁剪失效。
    // 需在 StartGrp 之后调用才生效; 检测结果在 post_thread 反偏移回整幅-AI 坐标。
    if (g_main_roi_w > 0.0f && g_main_roi_h > 0.0f) {
        // 反偏移系数: ROI-AI 坐标 -> 整幅-AI 坐标。
        // 通用公式: 整幅-AI 中 ROI 宽度(roi_w*g_ai_w) / chn1 实际宽度(g_chn_w)。
        // 因 g_chn_w 长边恒 = g_ai_w(640), 横向 ROI 时退化为 g_main_roi_w, 竖向 ROI 时正确放大。
        g_main_roi_ai_x0 = g_main_roi_x * g_ai_w;
        g_main_roi_ai_y0 = g_main_roi_y * g_ai_h;
        g_main_roi_ai_sx = g_main_roi_w * (float)g_ai_w / (float)g_chn_w;
        g_main_roi_ai_sy = g_main_roi_h * (float)g_ai_h / (float)g_chn_h;
    }

    // 车牌识别第三路 (源分辨率, 软件裁剪 ROI)
    if (g_car_cfg.enable) {
        memset(&cattr, 0, sizeof(cattr));
        cattr.enChnMode = VPSS_CHN_MODE_USER;
        cattr.enDynamicRange = DYNAMIC_RANGE_SDR8;
        cattr.enPixelFormat = RK_FMT_YUV420SP;
        cattr.u32Width = src_w;
        cattr.u32Height = src_h;
        cattr.u32Depth = 2;
        cattr.u32FrameBufCnt = 8;
        cattr.stMaxSize.u32Width = src_w;
        cattr.stMaxSize.u32Height = src_h;
        cattr.stFrameRate.s32SrcFrameRate = -1;
        cattr.stFrameRate.s32DstFrameRate = -1;
        cattr.enCompressMode = COMPRESS_MODE_NONE;
        RK_MPI_VPSS_SetChnAttr(0, g_car_cfg.chn, &cattr);
        RK_MPI_VPSS_EnableChn(0, g_car_cfg.chn);
    }

    ret = RK_MPI_VPSS_StartGrp(0);
    if (ret) { printf("VPSS start grp fail %x\n", ret); return -1; }

    // 主线程 ROI 硬件裁剪(StartGrp 之后设置才生效):
    // VPSS_CROP_ABS_COOR 绝对像素, 相对源图 src_w x src_h, 从 0-1 归一化换算。
    if (g_main_roi_w > 0.0f && g_main_roi_h > 0.0f) {
        VPSS_CROP_INFO_S crop;
        memset(&crop, 0, sizeof(crop));
        crop.bEnable = RK_TRUE;
        crop.enCropCoordinate = VPSS_CROP_ABS_COOR;
        crop.stCropRect.s32X = (RK_S32)(g_main_roi_x * src_w);
        crop.stCropRect.s32Y = (RK_S32)(g_main_roi_y * src_h);
        crop.stCropRect.u32Width  = (RK_U32)(g_main_roi_w * src_w);
        crop.stCropRect.u32Height = (RK_U32)(g_main_roi_h * src_h);
        RK_S32 cr = RK_MPI_VPSS_SetChnCrop(0, 1, &crop);
        printf("main roi: VPSS chn1 crop abs=(%d,%d %dx%d) norm=(%.3f,%.3f %.3fx%.3f) ret=%x\n",
               crop.stCropRect.s32X, crop.stCropRect.s32Y,
               crop.stCropRect.u32Width, crop.stCropRect.u32Height,
               g_main_roi_x, g_main_roi_y, g_main_roi_w, g_main_roi_h, cr);
        g_main_roi_enable = (cr == RK_SUCCESS) ? 1 : 0;
    }

    g_inited_vpss = 1;
    return 0;
}

// ---------------- 摄像头模式: VI(源分辨率) -> VPSS 分流 ----------------
static int camera_pipeline_init(void) {
    if (vi_dev_init() != 0) return -1;
    // VI chn0 输出源分辨率(4K), 深度0, 绑定到 VPSS
    if (vi_chn_init(0, g_vi_w, g_vi_h, 0) != 0) return -1;
    g_inited_vi = 1;
    if (vpss_pipeline_init(g_vi_w, g_vi_h) != 0) return -1;
    printf("camera pipeline: VI %dx%d -> VPSS %dx%d / %dx%d / %dx%d\n",
           g_vi_w, g_vi_h, g_main_w, g_main_h, g_ai_w, g_ai_h, g_vi_w, g_vi_h);
    return 0;
}

// ---------------- h264/h265 裸流 SPS 帧率自动解析 ----------------
// 裸流文件不带容器帧率信息, 但从 SPS 的 VUI timing_info 可读出真实帧率,
// 无需手动设置 file_fps, 播放速度自动匹配源视频。若 SPS 无 timing_info 则返回 0, 回退用配置值。
struct BitReader { const unsigned char *buf; int bitpos; int nbits; };
static inline int br_get_bit(struct BitReader *b) {
    if (b->bitpos >= b->nbits) return 0;
    int r = (b->buf[b->bitpos >> 3] >> (7 - (b->bitpos & 7))) & 1;
    b->bitpos++; return r;
}
static inline unsigned int br_get_bits(struct BitReader *b, int n) {
    unsigned int v = 0; for (int i = 0; i < n; i++) v = (v << 1) | br_get_bit(b); return v;
}
static inline unsigned int br_get_ue(struct BitReader *b) {
    int z = 0; while (!br_get_bit(b)) z++;
    return (1u << z) - 1 + (z ? br_get_bits(b, z) : 0);
}
static inline int br_get_se(struct BitReader *b) {
    unsigned int u = br_get_ue(b); return (int)((u & 1) ? (int)((u + 1) >> 1) : -(int)(u >> 1));
}
// 解析 h264 SPS VUI timing_info, 成功返回帧率, 否则返回 0
static double parse_h264_sps_fps(const unsigned char *p, int len) {
    if (len < 4) return 0.0;
    struct BitReader b = { p, 0, len * 8 };
    br_get_bits(&b, 8);  // profile_idc
    br_get_bits(&b, 8);  // constraint flags
    br_get_bits(&b, 8);  // level_idc
    br_get_ue(&b);       // seq_parameter_set_id
    int profile = p[0];
    if (profile == 100 || profile == 110 || profile == 122 || profile == 244 ||
        profile == 44 || profile == 83 || profile == 86 || profile == 118 ||
        profile == 128 || profile == 138 || profile == 139 || profile == 134) {
        int chroma = br_get_ue(&b);
        if (chroma == 3) br_get_bit(&b);          // separate_colour_plane_flag
        br_get_ue(&b);                            // bit_depth_luma_minus8
        br_get_ue(&b);                            // bit_depth_chroma_minus8
        br_get_bit(&b);                           // qpprime_y_zero_transform_bypass_flag
        if (br_get_bit(&b)) {                     // seq_scaling_matrix_present_flag
            for (int i = 0; i < 8; i++)
                if (br_get_bit(&b)) {             // seq_scaling_list_present_flag[i]
                    int size = (i < 6) ? 16 : 64, last = 8, next = 8;
                    for (int j = 0; j < size; j++) {
                        if (next) { int d = br_get_se(&b); next = (last + d + 256) % 256; }
                        last = next ? next : last;
                    }
                }
        }
    }
    br_get_ue(&b);       // log2_max_frame_num_minus4
    int poc = br_get_ue(&b);   // pic_order_cnt_type
    if (poc == 0) br_get_ue(&b);                  // log2_max_pic_order_cnt_lsb_minus4
    else if (poc == 1) {
        br_get_bit(&b);                           // delta_pic_order_always_zero_flag
        br_get_se(&b);                            // offset_for_non_ref_pic
        br_get_se(&b);                            // offset_for_top_to_bottom_field
        int n = br_get_ue(&b);                    // num_ref_frames_in_pic_order_cnt_cycle
        for (int i = 0; i < n; i++) br_get_se(&b);
    }
    br_get_ue(&b);       // max_num_ref_frames
    br_get_bit(&b);      // gaps_in_frame_num_value_allowed_flag
    unsigned int w_mbs = br_get_ue(&b);           // pic_width_in_mbs_minus1
    unsigned int h_mbs = br_get_ue(&b);           // pic_height_in_map_units_minus1
    (void)w_mbs; (void)h_mbs;
    unsigned int frame_mbs_only = br_get_bit(&b);
    if (!frame_mbs_only) br_get_bit(&b);          // mb_adaptive_frame_field_flag
    br_get_bit(&b);      // direct_8x8_inference_flag
    if (br_get_bit(&b)) { br_get_ue(&b); br_get_ue(&b); br_get_ue(&b); br_get_ue(&b); } // frame_cropping
    if (!br_get_bit(&b)) return 0.0;              // vui_parameters_present_flag
    // VUI: 仅需跳到 timing_info_present_flag
    if (br_get_bit(&b)) { br_get_ue(&b); br_get_ue(&b); br_get_bit(&b); br_get_bit(&b); } // aspect_ratio
    if (br_get_bit(&b)) { br_get_bit(&b); }       // overscan
    if (br_get_bit(&b)) {                         // video_signal_type
        br_get_bits(&b, 3); br_get_bit(&b);
        if (br_get_bit(&b)) { br_get_bits(&b, 8); br_get_bits(&b, 8); br_get_bits(&b, 8); } // colour_description
    }
    if (br_get_bit(&b)) { br_get_ue(&b); }        // chroma_loc_info
    if (!br_get_bit(&b)) return 0.0;              // timing_info_present_flag
    unsigned int num_units = br_get_bits(&b, 32);
    unsigned int time_scale = br_get_bits(&b, 32);
    int fixed = br_get_bit(&b);                   // fixed_frame_rate_flag
    if (num_units == 0) return 0.0;
    double f = (double)time_scale / (2.0 * num_units);   // h264 帧率公式 (frame 单位)
    if (!fixed) { /* 仍返回估算值 */ }
    return f;
}
// 解析 h265 SPS VUI timing_info, 成功返回帧率, 否则返回 0
static double parse_h265_sps_fps(const unsigned char *p, int len) {
    if (len < 4) return 0.0;
    struct BitReader b = { p, 0, len * 8 };
    br_get_bits(&b, 4);   // sps_video_parameter_set_id
    unsigned int sub_layers = br_get_bits(&b, 3);  // sps_max_sub_layers_minus1
    br_get_bit(&b);       // sps_temporal_id_nesting_flag
    // profile_tier_level(1): 8+32+4+43+1+sub_layers*(1+2+1+1) + (sub_layers?88:0)
    br_get_bits(&b, 2);   // general_profile_space
    br_get_bit(&b);       // general_tier_flag
    br_get_bits(&b, 5);   // general_profile_idc
    br_get_bits(&b, 32);  // general_profile_compatibility_flags
    br_get_bit(&b);       // general_progressive_source_flag
    br_get_bit(&b);       // general_interlaced_source_flag
    br_get_bit(&b);       // general_non_packed_constraint_flag
    br_get_bit(&b);       // general_frame_only_constraint_flag
    br_get_bits(&b, 43);  // reserved 43 bits
    for (unsigned int i = 0; i < sub_layers; i++) {
        br_get_bit(&b); br_get_bit(&b); br_get_bit(&b); br_get_bit(&b);  // sub_layer_profile_present...
    }
    if (sub_layers > 0) br_get_bits(&b, 88);  // reserved_zero_88bits
    br_get_ue(&b);       // sps_seq_parameter_set_id
    unsigned int chroma = br_get_ue(&b);      // chroma_format_idc
    if (chroma == 3) br_get_bit(&b);          // separate_colour_plane_flag
    br_get_ue(&b);       // pic_width_in_luma_samples
    br_get_ue(&b);       // pic_height_in_luma_samples
    if (br_get_bit(&b)) { br_get_ue(&b); br_get_ue(&b); br_get_ue(&b); br_get_ue(&b); } // conformance_window
    br_get_ue(&b);       // bit_depth_luma_minus8
    br_get_ue(&b);       // bit_depth_chroma_minus8
    br_get_ue(&b);       // log2_max_pic_order_cnt_lsb_minus4
    int sub_ordering = br_get_bit(&b);        // sps_sub_layer_ordering_info_present_flag
    unsigned int layers = sub_ordering ? 0 : sub_layers;
    for (unsigned int i = layers; i <= sub_layers; i++) {
        br_get_ue(&b); br_get_ue(&b); br_get_ue(&b);  // max_dec_pic_buffering/num_reorder/max_latency
    }
    br_get_ue(&b);       // log2_min_luma_coding_block_size_minus3
    br_get_ue(&b);       // log2_diff_max_min_luma_coding_block_size
    br_get_ue(&b);       // log2_min_transform_block_size_minus2
    br_get_ue(&b);       // log2_diff_max_min_transform_block_size
    br_get_ue(&b);       // max_transform_hierarchy_depth_inter
    br_get_ue(&b);       // max_transform_hierarchy_depth_intra
    if (br_get_bit(&b)) { /* scaling_list_enabled_flag */
        if (br_get_bit(&b)) { /* sps_scaling_list_data_present_flag */
            // scaling_list_data(): 4 组 (pred 模式列表 + DC 系数)
            for (int sz = 0; sz < 4; sz++) {
                for (int m = 0; m < (sz == 3 ? 2 : 6); m++) {
                    if (!br_get_bit(&b)) { br_get_se(&b); }  // scaling_list_pred_mode_flag
                    else { br_get_ue(&b); /* scaling_list_pred_matrix_id_delta */ }
                }
            }
        }
    }
    br_get_bit(&b);       // amp_enabled_flag
    br_get_bit(&b);       // sample_adaptive_offset_enabled_flag
    if (br_get_bit(&b)) { /* pcm_enabled_flag */
        br_get_bits(&b, 4); br_get_bits(&b, 4); br_get_ue(&b); br_get_ue(&b); br_get_bit(&b);
    }
    br_get_ue(&b);       // num_short_term_ref_pic_sets
    if (!br_get_bit(&b)) return 0.0;          // long_term_ref_pics_present_flag? no -- must keep going
    // NOTE: 上面是 long_term_ref_pics_present_flag, 继续
    if (br_get_bit(&b)) { /* sps_temporal_mvp_enabled_flag */ }
    br_get_bit(&b);      // strong_intra_smoothing_enabled_flag
    if (!br_get_bit(&b)) return 0.0;          // vui_parameters_present_flag
    // VUI
    if (br_get_bit(&b)) { br_get_ue(&b); br_get_ue(&b); br_get_bit(&b); } // aspect_ratio
    if (br_get_bit(&b)) { br_get_bit(&b); }   // overscan
    if (br_get_bit(&b)) { br_get_bits(&b, 3); br_get_bit(&b); if (br_get_bit(&b)) { br_get_bits(&b,8); br_get_bits(&b,8); br_get_bits(&b,8); } }
    if (br_get_bit(&b)) { br_get_ue(&b); }    // chroma_loc_info
    br_get_bits(&b, 3);   // neutral_chroma_indication(1) + field_seq_flag(1) + frame_field_info_present(1) — 无条件 3 位
    if (!br_get_bit(&b)) return 0.0;          // timing_info_present_flag
    unsigned int num_units = br_get_bits(&b, 32);
    unsigned int time_scale = br_get_bits(&b, 32);
    br_get_bit(&b);       // poc_proportional_to_timing_flag
    if (num_units == 0) return 0.0;
    return (double)time_scale / (double)num_units;   // h265 帧率公式
}
// 从 h264/h265 裸流文件扫描首个 SPS NAL, 返回自动解析的帧率; 找不到/解析失败返回 0
static double probe_file_framerate(const char *path, RK_CODEC_ID_E codec) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0.0;
    unsigned char buf[2 * 1024 * 1024];
    ssize_t n = read(fd, buf, sizeof(buf));
    close(fd);
    if (n <= 0) return 0.0;
    // 扫描 start code, 定位 NAL header (00 00 00 01 或 00 00 01)
    for (int i = 0; i < n - 4; i++) {
        int sc = (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1);
        if (!sc) continue;
        int hdr_pos = i + 3;                      // 3 字节 start code 后是 NAL header
        // 检查 4 字节 start code (前一个也是 0)
        int sc4 = (i > 0 && buf[i-1] == 0);
        if (sc4) hdr_pos = i + 3;                 // 00 00 00 01: header 在 i+3  (i+1,i+2 为 0, i+3=1)
        unsigned char hdr = buf[hdr_pos];
        if (codec == RK_VIDEO_ID_HEVC) {
            // h265 NAL unit type = (hdr >> 1) & 0x3f; SPS = 33
            if (((hdr >> 1) & 0x3f) == 33) {
                int sps_len = n - hdr_pos - 1;
                if (sps_len < 0) sps_len = 0;
                return parse_h265_sps_fps(buf + hdr_pos + 1, sps_len);
            }
        } else {
            // h264 NAL type = hdr & 0x1f; SPS = 7
            if ((hdr & 0x1f) == 7) {
                int sps_len = n - hdr_pos - 1;
                if (sps_len < 0) sps_len = 0;
                return parse_h264_sps_fps(buf + hdr_pos + 1, sps_len);
            }
        }
    }
    return 0.0;
}

// ---------------- 文件模式: VDEC 解码 + VPSS 分流 ----------------
static int file_pipeline_init(void) {
    int ret;

    // 自动从 SPS 解析真实帧率, 覆盖 file_fps, 播放速度匹配源视频(用户无需手动设置)。
    // 只有 SPS 含 timing_info 才覆盖; 否则回退用 config 的 file_fps。
    double auto_fps = probe_file_framerate(g_video_file, g_dec_codec);
    if (auto_fps > 1.0 && auto_fps < 100.0) {
        int old = g_file_fps;
        g_file_fps = (int)(auto_fps + 0.5);
        printf("[SPS-AUTO] probed %.2f fps from %s, file_fps %d -> %d\n",
               auto_fps, g_video_file, old, g_file_fps);
    } else {
        printf("[SPS-AUTO] no timing_info in %s, using file_fps=%d\n", g_video_file, g_file_fps);
    }

    // VDEC 模块参数: 手动分发(非绑定)模式必须用 MB_SOURCE_USER + 外部 MB 帧缓冲池,
    // 否则 VDEC 帧缓冲未正确配置, 解出的帧为空(仅 SPS 分辨率正确, YUV 内容全 0)。
    // 参考官方 sample_comm_vdec.c SAMPLE_COMM_VDEC_CreateChn。
    VDEC_MOD_PARAM_S vdec_mod_param;
    memset(&vdec_mod_param, 0, sizeof(vdec_mod_param));
    vdec_mod_param.enVdecMBSource = MB_SOURCE_USER;
    ret = RK_MPI_VDEC_SetModParam(&vdec_mod_param);
    if (ret != RK_SUCCESS) { printf("VDEC SetModParam fail %x\n", ret); return -1; }

    // VDEC 通道(帧模式)
    VDEC_CHN_ATTR_S vattr;
    memset(&vattr, 0, sizeof(vattr));
    vattr.enType = g_dec_codec;
    vattr.enMode = VIDEO_MODE_STREAM;
    vattr.u32PicWidth  = g_dec_w;
    vattr.u32PicHeight = g_dec_h;
    vattr.u32FrameBufCnt = 6;
    vattr.u32StreamBufCnt = 8;
    vattr.u32FrameBufDepth = 6;

    // 创建 VDEC 帧缓冲池(官方 MB_SOURCE_USER 流程)
    VDEC_PIC_BUF_ATTR_S stPicAttr;
    memset(&stPicAttr, 0, sizeof(stPicAttr));
    stPicAttr.enCodecType = g_dec_codec;
    stPicAttr.stPicBufAttr.u32Width = g_dec_w;
    stPicAttr.stPicBufAttr.u32Height = g_dec_h;
    stPicAttr.stPicBufAttr.enPixelFormat = (PIXEL_FORMAT_E)vattr.enPixelFormat;
    #ifdef RV1126B
    stPicAttr.stPicBufAttr.enCompMode = COMPRESS_MODE_NONE;
    #endif
    MB_PIC_CAL_S stMbCal;
    memset(&stMbCal, 0, sizeof(stMbCal));
    ret = RK_MPI_CAL_VDEC_GetPicBufferSize(&stPicAttr, &stMbCal);
    if (ret != RK_SUCCESS) { printf("VDEC GetPicBufferSize fail %x\n", ret); return -1; }

    MB_POOL_CONFIG_S stPoolCfg;
    memset(&stPoolCfg, 0, sizeof(stPoolCfg));
    stPoolCfg.u64MBSize = stMbCal.u32MBSize;
    stPoolCfg.u32MBCnt = 10;
    stPoolCfg.enRemapMode = MB_REMAP_MODE_CACHED;
    stPoolCfg.bPreAlloc = RK_TRUE;
    g_vdec_pool = RK_MPI_MB_CreatePool(&stPoolCfg);
    if (g_vdec_pool == MB_INVALID_POOLID) { printf("VDEC MB CreatePool fail\n"); return -1; }

    ret = RK_MPI_VDEC_CreateChn(0, &vattr);
    if (ret) { printf("VDEC create fail %x\n", ret); return -1; }

    // 官方样例在 CreateChn 后必调 SetChnParam(stParam.enType=codec), 对齐官方 SAMPLE_COMM_VDEC_CreateChn
    VDEC_CHN_PARAM_S vparam;
    memset(&vparam, 0, sizeof(vparam));
    vparam.enType = g_dec_codec;
    ret = RK_MPI_VDEC_SetChnParam(0, &vparam);
    if (ret) { printf("VDEC SetChnParam fail %x\n", ret); return -1; }

    ret = RK_MPI_VDEC_AttachMbPool(0, g_vdec_pool);
    if (ret != RK_SUCCESS) { printf("VDEC AttachMbPool fail %x\n", ret); return -1; }
    RK_MPI_VDEC_StartRecvStream(0);
    g_inited_vdec = 1;

    // VPSS 分流: chn0 主码流 -> VENC, chn1 AI, chn2 车牌(源分辨率)
    if (vpss_pipeline_init(g_dec_w, g_dec_h) != 0) return -1;

    // VDEC->VPSS 采用手动分发线程 (vdec_to_vpss_thread): GetFrame -> SendFrame -> ReleaseFrame
    printf("file pipeline: %s (%dx%d) -> VPSS %dx%d / %dx%d\n",
           g_video_file, g_dec_w, g_dec_h, g_main_w, g_main_h, g_ai_w, g_ai_h);
    return 0;
}

// 逐 NAL 切一帧: 返回该帧字节数, 并把文件位置留在帧结束处; eos 置位表示到文件尾。
// h264: SPS/PPS/SEI 等非帧 NAL 跳过, 遇到 IDR/SLICE(frameType 5/1) 才开始累计, 到下一帧 start code 停止。
static unsigned h264_frame_size(int fd, unsigned char nal[5], int *eos) {
    int frameType = 0, isNalu = 0, isFrame = 0;
    unsigned size = 0;
    off_t filesize, pos = lseek(fd, 0, SEEK_CUR);
    filesize = lseek(fd, 0, SEEK_END);
    lseek(fd, pos, SEEK_SET);
    while (1) {
        nal[0]=nal[1]; nal[1]=nal[2]; nal[2]=nal[3]; nal[3]=nal[4];
        if (read(fd, &nal[4], 1) != 1) {
            *eos = (lseek(fd,0,SEEK_CUR)==filesize);
            lseek(fd, pos, SEEK_SET); return size;
        }
        size++;
        if (isNalu) { isNalu=0; frameType = nal[4]&0x1F; if (frameType==5||frameType==1) isFrame=1; }
        if ((nal[0]!=0x00 && nal[1]==0x00 && nal[2]==0x00 && nal[3]==0x01) ||
            (nal[1]==0x00 && nal[2]==0x00 && nal[3]==0x00 && nal[4]==0x01)) {
            if (isFrame) {
                size = nal[3]==0x01 ? size-3 : size-4;
                *eos = (lseek(fd,0,SEEK_CUR)==filesize);
                lseek(fd, pos, SEEK_SET); return size;
            } else isNalu = 1;
        }
    }
}
// h265 版本
static unsigned h265_frame_size(int fd, unsigned char nal[5], int *eos) {
    int frameType = 0, isNalu = 0, isFrame = 0;
    unsigned size = 0;
    off_t filesize, pos = lseek(fd, 0, SEEK_CUR);
    filesize = lseek(fd, 0, SEEK_END);
    lseek(fd, pos, SEEK_SET);
    while (1) {
        nal[0]=nal[1]; nal[1]=nal[2]; nal[2]=nal[3]; nal[3]=nal[4];
        if (read(fd, &nal[4], 1) != 1) {
            *eos = (lseek(fd,0,SEEK_CUR)==filesize);
            lseek(fd, pos, SEEK_SET); return size;
        }
        size++;
        if (isNalu) {
            if (!frameType) { frameType = nal[4]<<8; continue; }
            else { isNalu=0; frameType |= nal[4]; }
            if (frameType==0x2601 || frameType==0x0201) isFrame=1;
        }
        if ((nal[0]!=0x00 && nal[1]==0x00 && nal[2]==0x00 && nal[3]==0x01) ||
            (nal[1]==0x00 && nal[2]==0x00 && nal[3]==0x00 && nal[4]==0x01)) {
            if (isFrame) {
                size = nal[3]==0x01 ? size-3 : size-4;
                *eos = (lseek(fd,0,SEEK_CUR)==filesize);
                lseek(fd, pos, SEEK_SET); return size;
            } else { frameType=0; isNalu=1; }
        }
    }
}

static void *file_feed_thread(void *arg) {
    (void)arg;
    int fd = open(g_video_file, O_RDONLY);
    if (fd < 0) { printf("open video fail: %s\n", g_video_file); g_quit = true; return NULL; }
    int is265 = (g_dec_codec == RK_VIDEO_ID_HEVC);
    // 源端按帧限速: 每喂一帧等一帧节拍。这是根治"画面加速"的关键——紧循环喂流会让 VDEC
    // 超速解码积压, VPSS chn0/VENC 绑定后因缓冲满而丢帧, 相邻 VENC 输出帧画面跳变(快进)。
    // 高效实现: 大块一次读入内存, 在内存里切帧(避免逐字节 read 系统调用开销), 再按精确节拍喂给 VDEC。
    int fps = (g_file_fps > 0) ? g_file_fps : 25;
    int64_t frame_ns = 1000000000LL / fps;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    // 滚动读缓冲: 尽量读满, 在内存中定位帧边界。
    #define FBUF_CAP (8 * 1024 * 1024)
    unsigned char *buf = (unsigned char *)malloc(FBUF_CAP);
    if (!buf) { printf("feed: malloc fail\n"); close(fd); g_quit = true; return NULL; }
    size_t bstart = 0, bvalid = 0;      // [bstart, bvalid) 为有效数据
    int sent = 0;

    // 从有效区起点扫描: 定位下一视频帧 [fstart, fend), 返回 1 找到完整帧;
    // 返回 0 数据耗尽(需重读), 返回 -1 已到文件尾。
    // 状态机镜像原 h264_frame_size/h265_frame_size (已证正确): 跳过 SPS/PPS/SEI 等非帧 NAL,
    // 遇到 IDR(5)/SLICE(1) 或 h265(0x26/0x02) 才置 isFrame, 到下一视频帧 start code 前为本帧结束。
    auto next_frame = [&](size_t *fstart, size_t *fend) -> int {
        size_t pos = bstart;
        unsigned char b0=0,b1=0,b2=0,b3=0,b4=0;   // 滑动 5 字节窗 (b0 最新)
        int frameType = 0, isNalu = 0, isFrame = 0;
        while (1) {
            if (pos >= bvalid) {          // 数据耗尽
                *fstart = bstart; *fend = bvalid;
                return 0;
            }
            b4=b3; b3=b2; b2=b1; b1=b0; b0 = buf[pos++];   // 左移: b0=新字节
            if (isNalu) {
                if (is265) {
                    if (!frameType) { frameType = b0<<8; continue; }  // 第1字节, isNalu 保留
                    isNalu = 0; frameType |= b0;                      // 第2字节
                } else {
                    isNalu = 0;
                    frameType = b0 & 0x1F;
                }
                if (is265 ? (frameType==0x2601 || frameType==0x0201)
                          : (frameType==5 || frameType==1)) isFrame = 1;
            }
            int sc3 = (b3!=0 && b2==0 && b1==0 && b0==1);   // 3字节 "00 00 01": 末字节=b0, b3!=0 防 4字节
            int sc4 = (b3==0 && b2==0 && b1==0 && b0==1);   // 4字节 "00 00 00 01": 末字节=b0
            if (sc4 || sc3) {
                if (isFrame) {
                    *fstart = bstart;
                    *fend = pos - (sc4 ? 4 : 3);   // 排除 start code 本身
                    return 1;
                } else {
                    frameType = 0;
                    isNalu = 1;
                }
            }
        }
    };

    auto send_frame = [&](size_t fstart, size_t fend) {
        size_t fsize = fend - fstart;
        MB_BLK mb;
        if (RK_MPI_MMZ_Alloc(&mb, (RK_U32)fsize, 0) != RK_SUCCESS) return;
        memcpy(RK_MPI_MB_Handle2VirAddr(mb), buf + fstart, fsize);
        VDEC_STREAM_S st;
        memset(&st, 0, sizeof(st));
        st.pMbBlk = mb;
        st.u32Len = (RK_U32)fsize;
        st.u64PTS = 0;
        st.bEndOfStream = RK_FALSE;
        st.bEndOfFrame = RK_FALSE;
        st.bBypassMbBlk = RK_TRUE;
        RK_S32 sr = RK_MPI_VDEC_SendStream(0, &st, 1000);
        if (sr != RK_SUCCESS && sent < 8)
            printf("feed: SendStream ret=%x\n", sr);
        sent++;
        if (sent <= 5 || (sent % 100) == 0) printf("feed frame %d size %zu\n", sent, fsize);
        RK_MPI_MMZ_Free(mb);
    };

    while (!g_quit) {
        size_t fstart, fend;
        int r = next_frame(&fstart, &fend);
        if (r == 1) {
            send_frame(fstart, fend);
            bstart = fend;
            // 精确节拍限速: 每帧等一帧时长, 落后则追平不累积
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            int64_t nowns = (int64_t)now.tv_sec*1000000000LL + now.tv_nsec;
            int64_t tgt = (int64_t)next.tv_sec*1000000000LL + next.tv_nsec;
            int64_t sl = tgt - nowns;
            if (sl > 0) {
                struct timespec req = { (time_t)(sl/1000000000LL), (long)(sl%1000000000LL) };
                struct timespec rem;
                clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &rem);
            }
            int64_t base = (int64_t)next.tv_sec*1000000000LL + next.tv_nsec + frame_ns;
            clock_gettime(CLOCK_MONOTONIC, &now);
            nowns = (int64_t)now.tv_sec*1000000000LL + now.tv_nsec;
            if (base < nowns) base = nowns;
            next.tv_sec = (time_t)(base/1000000000LL);
            next.tv_nsec = (long)(base%1000000000LL);
        } else {
            // 数据耗尽(当前缓冲无完整帧): 压实后尽量补读。若缓冲已满(>1MB 仍无完整帧, 罕见大尾帧),
            // 说明剩余全是无法闭合的截断数据, 直接丢弃并循环回文件头。
            if (bvalid - bstart >= (1<<20)) {
                lseek(fd, 0, SEEK_SET);
                bstart = 0; bvalid = 0;
                printf("video loop (truncated tail)\n");
                usleep(5000);
                continue;
            }
            if (bstart) { memmove(buf, buf + bstart, bvalid - bstart); bvalid -= bstart; bstart = 0; }
            ssize_t rr = read(fd, buf + bvalid, FBUF_CAP - bvalid);
            if (rr <= 0) {                  // 到文件尾 -> 循环回开头
                lseek(fd, 0, SEEK_SET);
                bstart = 0; bvalid = 0;
                printf("video loop\n");
                usleep(5000);
                continue;
            }
            bvalid += (size_t)rr;
        }
    }
    free(buf);
    close(fd);
    return NULL;
}

// 手动分发线程: VDEC 解码帧 -> VPSS grp0 (pipe0)。
// 文件模式不采用 VDEC->VPSS 硬件绑定 (RV1126 上绑定后 VPSS 不从 VDEC 拉流, chn1 无帧), 改为
// 官方标准 GetFrame -> SendFrame -> ReleaseFrame 手动投递, 使 VPSS chn0/chn1/chn2 正常分流。
static void *vdec_to_vpss_thread(void *arg) {
    (void)arg;
    printf("[VDEC->VPSS] manual dispatch thread start\n");
    int n = 0, nfail = 0;
    // 播放速度由源端喂流按 file_fps 限速(file_feed_thread 每帧 usleep), VDEC 即按源帧率解码,
    // 无积压无丢帧。此处只做 GetFrame->SendFrame->ReleaseFrame 手动投递, 不再二次限速(避免抖动)。
    while (!g_quit) {
        VIDEO_FRAME_INFO_S frame;
        memset(&frame, 0, sizeof(frame));
        RK_S32 gr = RK_MPI_VDEC_GetFrame(0, &frame, 1000);
        if (gr != RK_SUCCESS) continue;
        if (n < 8) {
            // MB_REMAP_MODE_CACHED 池: VDEC 通过 DMA 写解码帧, CPU 需先失效 cache 才能读到真实数据
            RK_MPI_SYS_MmzFlushCache(frame.stVFrame.pMbBlk, RK_FALSE);
            void *vv = RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
            unsigned char *pp = (unsigned char *)vv;
            int vz = 0; unsigned sum = 0;
            if (pp) for (int i = 0; i < 4096; i++) { if (pp[i]) vz++; sum += pp[i]; }
            printf("[VDEC] out[%d] fmt=%d %ux%u vir=%ux%u nz=%d/4096 sum=%u data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                   n, (int)frame.stVFrame.enPixelFormat,
                   (unsigned)frame.stVFrame.u32Width, (unsigned)frame.stVFrame.u32Height,
                   (unsigned)frame.stVFrame.u32VirWidth, (unsigned)frame.stVFrame.u32VirHeight,
                   vz, sum, pp ? pp[0] : 0, pp ? pp[1] : 0, pp ? pp[2] : 0, pp ? pp[3] : 0,
                   pp ? pp[4] : 0, pp ? pp[5] : 0, pp ? pp[6] : 0, pp ? pp[7] : 0);
        }
        n++;
        RK_S32 sr = RK_MPI_VPSS_SendFrame(0, 0, &frame, 1000);
        if (sr != RK_SUCCESS) {
            nfail++;
            if (nfail <= 8)
                printf("[VDEC->VPSS] SendFrame fail ret=0x%x (total %d)\n", sr, nfail);
        }
        RK_MPI_VDEC_ReleaseFrame(0, &frame);
    }
    return NULL;
}

// ---------------- RGN 全屏画框 ----------------
static int rgn_init(void) {
    int ret;
    RGN_ATTR_S rgn_attr;
    RGN_CHN_ATTR_S chn_attr;
    MPP_CHN_S mpp;

    // overlay 尺寸需与主码流视频完全一致(不可对高度做 16 对齐):
    // 若 canvas 高=ALIGN16(1080)=1088 而视频=1080, 驱动会把 1088 canvas 垂直缩放到 1080,
    // 导致每个绘制元素的底部若干行被吃掉(文字下半截/大写字母底横消失)。
    // 1920 宽本身 16 对齐, 高度 1080 直接给视频高度即可(驱动内部自行处理 stride 对齐)。
    g_ov_w = g_main_w;
    g_ov_h = g_main_h;

    memset(&rgn_attr, 0, sizeof(rgn_attr));
    rgn_attr.enType = OVERLAY_RGN;
    rgn_attr.unAttr.stOverlay.enPixelFmt = (PIXEL_FORMAT_E)RK_FMT_ARGB8888;
    rgn_attr.unAttr.stOverlay.stSize.u32Width = g_ov_w;
    rgn_attr.unAttr.stOverlay.stSize.u32Height = g_ov_h;
    rgn_attr.unAttr.stOverlay.u32CanvasNum = 2;
    ret = RK_MPI_RGN_Create(g_rgn, &rgn_attr);
    if (ret) { printf("RGN create fail %x\n", ret); return -1; }

    memset(&chn_attr, 0, sizeof(chn_attr));
    chn_attr.bShow = RK_TRUE;
    chn_attr.enType = OVERLAY_RGN;
    chn_attr.unChnAttr.stOverlayChn.stPoint.s32X = 0;
    chn_attr.unChnAttr.stOverlayChn.stPoint.s32Y = 0;
    chn_attr.unChnAttr.stOverlayChn.u32BgAlpha = 0;
    chn_attr.unChnAttr.stOverlayChn.u32FgAlpha = 255;
    chn_attr.unChnAttr.stOverlayChn.u32Layer = 1;
    chn_attr.unChnAttr.stOverlayChn.stQpInfo.bEnable = RK_FALSE;
    // bForceIntra 强制 RGN 区域每帧按 I 块编码, 本 SDK 上会导致叠加区局部马赛克,
    // 故关闭, 让叠加区随帧间预测正常编码 (文本清晰度略有下降但画面干净)。
    chn_attr.unChnAttr.stOverlayChn.stQpInfo.bForceIntra = RK_FALSE;

    mpp.enModId = RK_ID_VENC; mpp.s32DevId = 0; mpp.s32ChnId = 0;
    ret = RK_MPI_RGN_AttachToChn(g_rgn, &mpp, &chn_attr);
    if (ret) { printf("RGN attach fail %x\n", ret); return -1; }
    printf("RGN overlay ready %dx%d (canvas mode)\n", g_ov_w, g_ov_h);
    return 0;
}

// RV1126B ARGB8888 内存实际为 BGRA 字序(官方样例注明), 颜色按 RK 宏:
//   RED=0x0000FFFF GREEN=0x00FF00FF BLUE=0xFF0000FF WHITE=0xFFFFFFFF
// 32 位字: [0]=B [1]=G [2]=R [3]=A, 即 A 在最低字节。
// 颜色 alpha(RK overlay 中像素透明与否取决于 A 通道)须为 0xFF 才不透明显示。
#define COL_RED    0x0000FFFFu
#define COL_GREEN  0x00FF00FFu
#define COL_YELLOW 0x00FFFFFFu
#define COL_WHITE  0xFFFFFFFFu
#define COL_BLACK  0x000000FFu
#define COL_BG     0xC0000078u

static void draw_rect(RK_U32 *buf, int w, int h, int x, int y, int rw, int rh,
                      RK_U32 color, int thick) {
    for (int t = 0; t < thick; t++) {
        int y0 = y + t, y1 = y + rh - 1 - t;
        int x0 = x + t, x1 = x + rw - 1 - t;
        for (int xx = x0; xx <= x1; xx++) {
            if (y0 >= 0 && y0 < h && xx >= 0 && xx < w) buf[y0 * w + xx] = color;
            if (y1 >= 0 && y1 < h && xx >= 0 && xx < w) buf[y1 * w + xx] = color;
        }
        for (int yy = y0; yy <= y1; yy++) {
            if (yy >= 0 && yy < h && x0 >= 0 && x0 < w) buf[yy * w + x0] = color;
            if (yy >= 0 && yy < h && x1 >= 0 && x1 < w) buf[yy * w + x1] = color;
        }
    }
}

// 虚线矩形 (ROI 区域标识用): dash 为虚线每段长度, gap 为间隔长度
static void draw_rect_dashed(RK_U32 *buf, int w, int h, int x, int y, int rw, int rh,
                             RK_U32 color, int thick, int dash, int gap) {
    if (dash <= 0) dash = 8;
    if (gap <= 0) gap = 4;
    for (int t = 0; t < thick; t++) {
        int y0 = y + t, y1 = y + rh - 1 - t;
        int x0 = x + t, x1 = x + rw - 1 - t;
        // 上下边: 横向虚线
        for (int xx = x0; xx <= x1; ) {
            int e = xx + dash - 1 < x1 ? xx + dash - 1 : x1;
            for (int px = xx; px <= e; px++) {
                if (y0 >= 0 && y0 < h && px >= 0 && px < w) buf[y0 * w + px] = color;
                if (y1 >= 0 && y1 < h && px >= 0 && px < w) buf[y1 * w + px] = color;
            }
            xx = e + 1 + gap;
        }
        // 左右边: 纵向虚线
        for (int yy = y0; yy <= y1; ) {
            int e = yy + dash - 1 < y1 ? yy + dash - 1 : y1;
            for (int py = yy; py <= e; py++) {
                if (py >= 0 && py < h && x0 >= 0 && x0 < w) buf[py * w + x0] = color;
                if (py >= 0 && py < h && x1 >= 0 && x1 < w) buf[py * w + x1] = color;
            }
            yy = e + 1 + gap;
        }
    }
}

// ---- 文字绘制(内置5x7字体) ----
static void draw_pixel(RK_U32 *buf, int w, int h, int x, int y, RK_U32 color) {
    if (x >= 0 && x < w && y >= 0 && y < h) buf[y * w + x] = color;
}
static void draw_char(RK_U32 *buf, int w, int h, int x, int y, char ch,
                      RK_U32 color, int scale) {
    if (ch < 0x20 || ch > 0x7E) ch = ' ';
    const unsigned char *g = font5x7[ch - 0x20];
    for (int col = 0; col < 5; col++) {
        unsigned char bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (!(bits & (1 << row))) continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx, py = y + row * scale + sy;
                    // 之前每个字素周围画 1px 黑色描边, 小字号下黑色占主导,
                    // 导致文字看起来是黑色而非配置色, 故去掉描边, 直接画实色
                    draw_pixel(buf, w, h, px, py, color);
                }
        }
    }
}
static void draw_text(RK_U32 *buf, int w, int h, int x, int y, const char *s,
                      RK_U32 color, int scale) {
    int cx = x;
    int step = 6 * scale;
    for (; *s; s++) { draw_char(buf, w, h, cx, y, *s, color, scale); cx += step; }
}

// ---- 车牌号统一点阵绘制(中文省份字 + ASCII 数字字母同高 16x16) ----
// UTF-8 中文为 3 字节; ASCII 为 1 字节; 都查 plate_font 表, 单元宽度一致保证排版整齐
static const plate_glyph_t *find_plate_glyph(const char *s) {
    const unsigned char *p = (const unsigned char *)s;
    for (int i = 0; i < (int)(sizeof(plate_font) / sizeof(plate_font[0])); i++) {
        const char *u = plate_font[i].utf8;
        int L = (unsigned char)u[0] >= 0xC0 ? 3 : 1;   // 该表项是中文还是 ASCII
        int ok = 1;
        for (int k = 0; k < L; k++)
            if (p[k] != (unsigned char)u[k]) { ok = 0; break; }
        if (ok) return &plate_font[i];
    }
    return NULL;
}

// 从当前字节起该字符占几个 UTF-8 字节 (中文3, ASCII1)
static int glyph_utf8_len(const char *s) {
    const plate_glyph_t *g = find_plate_glyph(s);
    if (g) return (unsigned char)g->utf8[0] >= 0xC0 ? 3 : 1;
    return ((unsigned char)s[0] >= 0xC0) ? 3 : 1;
}

static void draw_plate_glyph(RK_U32 *buf, int w, int h, int x, int y,
                             const plate_glyph_t *g, RK_U32 color, int scale) {
    for (int row = 0; row < PLATE_H; row++) {
        int v = g->bmp[row * 2] | (g->bmp[row * 2 + 1] << 8);
        for (int col = 0; col < PLATE_W; col++) {
            if (!(v & (1 << col))) continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx, py = y + row * scale + sy;
                    // 底部边界用实际显示高度 g_main_h: 画布 chh(=1088) 比视频(=1080)高几行,
                    // 超出的行在合成/推流/保存时会被裁掉, 故按 1080 限制, 避免画出后被切底
                    if (px >= 0 && px < w && py >= 0 && py < h)
                        buf[py * w + px] = color;
                }
        }
    }
}

// 兜底字形: 字库找不到时画一个 16x16 实心方框轮廓, 保证该字符仍显示为可见文字
static void draw_plate_glyph_fallback(RK_U32 *buf, int w, int h, int x, int y,
                                      RK_U32 color, int scale) {
    for (int row = 0; row < PLATE_H; row++)
        for (int col = 0; col < PLATE_W; col++) {
            int edge = (row == 0 || row == PLATE_H - 1 || col == 0 || col == PLATE_W - 1);
            if (!edge) continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++) {
                    int px = x + col * scale + sx, py = y + row * scale + sy;
                    if (px >= 0 && px < w && py >= 0 && py < h)
                        buf[py * w + px] = color;
                }
        }
}

// 用统一车牌字体绘制 UTF-8 混合文本, 每字符等宽 PLATE_W*scale
// 字库找不到的字符用兜底方框代替, 保证车牌号每个字符都画出完整 16 行, 不再缺底
static int draw_plate_text(RK_U32 *buf, int w, int h, int x, int y,
                           const char *s, RK_U32 color, int scale) {
    int cx = x;
    for (const char *p = s; *p;) {
        const plate_glyph_t *g = find_plate_glyph(p);
        int adv = PLATE_W * scale;
        if (g) { draw_plate_glyph(buf, w, h, cx, y, g, color, scale); }
        else   { draw_plate_glyph_fallback(buf, w, h, cx, y, color, scale); }
        p += glyph_utf8_len(p);
        cx += adv;
    }
    return cx - x;
}

// 计算车牌字体文本像素宽度 (每字符等宽, 按 UTF-8 字符计数)
static int plate_text_width(const char *s, int scale) {
    int n = 0;
    for (const char *p = s; *p; p += glyph_utf8_len(p)) n++;
    return n * PLATE_W * scale;
}

// 把 ARGB8888 canvas (内存 BGRA: [0]=B [1]=G [2]=R [3]=A) 保存为 PPM 图
// 用于核对"绘制结果本身"是否完整, 以区分 RGN 合成/推流问题。
static void save_canvas_ppm(const char *path, const RK_U32 *buf, int w, int h, int stride) {
    FILE *f = fopen(path, "wb");
    if (!f) { printf("[SAVE] open %s fail\n", path); return; }
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    static unsigned char row[1920 * 3];
    for (int y = 0; y < h; y++) {
        const RK_U32 *src = buf + (size_t)y * stride;
        for (int x = 0; x < w; x++) {
            RK_U32 v = src[x];
            unsigned char b = (v >> 24) & 0xFF, g_ = (v >> 16) & 0xFF, r = (v >> 8) & 0xFF;
            row[x * 3 + 0] = r; row[x * 3 + 1] = g_; row[x * 3 + 2] = b;
        }
        fwrite(row, 1, w * 3, f);
    }
    fclose(f);
    printf("[SAVE] wrote %s (%dx%d)\n", path, w, h);
}

// 显示层 ID 平滑: ByteTrack/DeepSORT 偶发对某帧返回 track_id=-1 (同目标), 若直接不画会闪烁。
// 这里记住每个框最近的 track_id, 当某帧 track_id 缺失时用 IoU 匹配回最近 ID, 保持文字稳定。
#define ID_HOLD_MAX 32
#define ID_HOLD_WIN_SEC 1      // 缺失后最多保持该秒数
static struct {
    float cx, cy, w, h;   // AI 坐标框
    int   id;             // 最近有效 track_id
    long  last_ms;        // 最近更新时间 (毫秒)
} g_id_hold[ID_HOLD_MAX];

static float hold_iou(float ax, float ay, float aw, float ah,
                      float bx, float by, float bw, float bh) {
    float x1 = ax > bx ? ax : bx, y1 = ay > by ? ay : by;
    float x2 = (ax + aw) < (bx + bw) ? (ax + aw) : (bx + bw);
    float y2 = (ay + ah) < (by + bh) ? (ay + ah) : (by + bh);
    float inter = (x2 - x1 > 0 ? x2 - x1 : 0) * (y2 - y1 > 0 ? y2 - y1 : 0);
    float uni = aw * ah + bw * bh - inter;
    return uni <= 0 ? 0.f : inter / uni;
}

// 取某检测框的稳定显示 ID: 优先用当前 track_id, 缺失则用历史 IoU 匹配
static int stable_id(const object_detect_result *r) {
    long long now;
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    now = (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    float cx = r->box.left, cy = r->box.top;
    float cw = r->box.right - r->box.left, ch = r->box.bottom - r->box.top;
    if (r->track_id >= 0) {
        int slot = r->track_id % ID_HOLD_MAX;
        g_id_hold[slot].cx = cx; g_id_hold[slot].cy = cy;
        g_id_hold[slot].w = cw;  g_id_hold[slot].h = ch;
        g_id_hold[slot].id = r->track_id; g_id_hold[slot].last_ms = now;
        return r->track_id;
    }
    int best = -1; float best_iou = 0;
    for (int i = 0; i < ID_HOLD_MAX; i++) {
        if (g_id_hold[i].id < 0) continue;
        if ((now - g_id_hold[i].last_ms) > (long long)ID_HOLD_WIN_SEC * 1000) { g_id_hold[i].id = -1; continue; }
        float iou = hold_iou(cx, cy, cw, ch, g_id_hold[i].cx, g_id_hold[i].cy, g_id_hold[i].w, g_id_hold[i].h);
        if (iou > best_iou) { best_iou = iou; best = i; }
    }
    return (best >= 0 && best_iou > 0.3f) ? g_id_hold[best].id : -1;
}

// 用最新检测结果刷新 overlay (整幅-AI 坐标系 g_ai_w x g_ai_h -> 主码流 1920x1080)
static void rgn_update(const object_detect_result_list *od) {
    g_have_plate_text = 0;
    RGN_CANVAS_INFO_S ci;
    memset(&ci, 0, sizeof(ci));
    if (RK_MPI_RGN_GetCanvasInfo(g_rgn, &ci) != RK_SUCCESS) return;
    RK_U32 *buf = (RK_U32 *)(uintptr_t)ci.u64VirAddr;
    int stride = (int)ci.u32VirWidth;   // 像素行距
    int cw = (int)ci.u32VirWidth, chh = (int)ci.u32VirHeight;
    for (int yy = 0; yy < chh; yy++)
        memset(buf + (size_t)yy * stride, 0, (size_t)cw * 4);

    float sx = (float)g_main_w / g_ai_w;
    float sy = (float)g_main_h / g_ai_h;
    int font_scale = g_osd_scale;  // 可配置 (5x7*scale)

    // 检测框 + 类名标签
    for (int i = 0; i < od->count; i++) {
        const object_detect_result *r = &od->results[i];
        int x  = (int)(r->box.left   * sx);
        int y  = (int)(r->box.top    * sy);
        int x2 = (int)(r->box.right  * sx);
        int y2 = (int)(r->box.bottom * sy);
        RK_U32 c = (r->cls_id == 1) ? COL_RED : COL_GREEN;  // person(cls1)红, 其余绿
        draw_rect(buf, stride, g_main_h, x, y, x2 - x, y2 - y, c, 4);
        char label[64];
        int sid = stable_id(r);
        if (sid >= 0)
            snprintf(label, sizeof(label), "%s %.2f #%d", cls_to_name(r->cls_id), r->prop, sid);
        else
            snprintf(label, sizeof(label), "%s %.2f", cls_to_name(r->cls_id), r->prop);
        // 车牌/车辆识别信息追加到标签 (车身色 + 车牌号, ASCII; 中文省份字在 5x7 字库显示为空格,
        // 完整中文车牌见下方独立车牌框的 draw_plate_text)
        if (r->has_plate || r->car_color[0]) {
            char extra[128] = "";
            if (g_show_plate && r->has_plate && r->plate_number[0])
                snprintf(extra, sizeof(extra), " %s", r->plate_number);
            if (r->car_color[0])
                snprintf(extra + strlen(extra), sizeof(extra) - strlen(extra), " %s", r->car_color);
            strncat(label, extra, sizeof(label) - strlen(label) - 1);
        }
        int tw = (int)strlen(label) * 6 * font_scale;
        int ly = y - 11 * font_scale;  if (ly < 0) ly = y;
        // 底部边界用实际主码流高度 g_main_h (画布 ALIGN16 比显示高几行, 会被编码裁剪)
        for (int yy = ly; yy < ly + 11 * font_scale && yy < g_main_h; yy++)
            for (int xx = x; xx < x + tw + 6 && xx < cw; xx++)
                if (xx >= 0 && yy >= 0) buf[yy * stride + xx] = COL_BG;
        draw_text(buf, stride, g_main_h, x + 3, ly + 2, label, COL_YELLOW, font_scale);
    }

    // 日期时间 + 帧率/目标数 (左上角 OSD)
    if (g_show_osd || g_show_fps) {
        char line[128];
        time_t now = time(NULL);
        struct tm tmv;
        localtime_r(&now, &tmv);
        char tbuf[32];
        strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tmv);
        if (g_show_osd && g_show_fps)
            snprintf(line, sizeof(line), "%s  %.1ffps obj:%d", tbuf, g_fps, od->count);
        else if (g_show_osd)
            snprintf(line, sizeof(line), "%s", tbuf);
        else
            snprintf(line, sizeof(line), "%.1ffps obj:%d", g_fps, od->count);
        int tw = (int)strlen(line) * 6 * font_scale;
        for (int yy = 6; yy < 6 + 11 * font_scale && yy < chh; yy++)
            for (int xx = 10; xx < 10 + tw + 8 && xx < cw; xx++)
                buf[yy * stride + xx] = COL_BG;
        draw_text(buf, stride, g_main_h, 14, 8, line, g_osd_color, font_scale);
    }

    // 车牌识别 ROI 框 (源帧 4K 坐标 -> 主码流坐标, 红色虚线)
    if (g_car_cfg.enable && g_car_cfg.roi_w > 0 && g_car_cfg.roi_h > 0 &&
        g_vi_w > 0 && g_vi_h > 0) {
        float rxs = (float)g_main_w / (float)g_vi_w;
        float rys = (float)g_main_h / (float)g_vi_h;
        int rx  = (int)(g_car_cfg.roi_x * rxs);
        int ry  = (int)(g_car_cfg.roi_y * rys);
        int rx2 = (int)((g_car_cfg.roi_x + g_car_cfg.roi_w) * rxs);
        int ry2 = (int)((g_car_cfg.roi_y + g_car_cfg.roi_h) * rys);
        draw_rect_dashed(buf, stride, g_main_h, rx, ry, rx2 - rx, ry2 - ry, COL_RED, 2, 8, 4);
        draw_text(buf, stride, g_main_h, rx + 3, ry + 2, "ROI", COL_RED, font_scale);
    }

    // 主线程检测 ROI 框 (整幅-AI 坐标 -> 主码流坐标, 绿色虚线)
    // 配置了 main_roi(>0) 即绘制, 便于直观看到裁剪放大检测区域
    if (g_main_roi_w > 0.0f && g_main_roi_h > 0.0f && g_ai_w > 0 && g_ai_h > 0) {
        float mxs = (float)g_main_w / (float)g_ai_w;
        float mys = (float)g_main_h / (float)g_ai_h;
        int mx  = (int)(g_main_roi_ai_x0 * mxs);
        int my  = (int)(g_main_roi_ai_y0 * mys);
        int mx2 = (int)((g_main_roi_ai_x0 + g_main_roi_w * g_ai_w) * mxs);
        int my2 = (int)((g_main_roi_ai_y0 + g_main_roi_h * g_ai_h) * mys);
        draw_rect_dashed(buf, stride, g_main_h, mx, my, mx2 - mx, my2 - my, COL_GREEN, 2, 8, 4);
        draw_text(buf, stride, g_main_h, mx + 3, my + 2, "MROI", COL_GREEN, font_scale);
    }

    // 车牌检测框 (源帧 4K 坐标 -> 主码流坐标, 蓝框 + 车牌号)
    // 受 show_plate 开关控制: 关闭时不绘制车牌检测框, 也不绘制车牌号文字
    if (g_show_plate && g_car_cfg.enable && g_vi_w > 0 && g_vi_h > 0) {
        float pxs = (float)g_main_w / (float)g_vi_w;
        float pys = (float)g_main_h / (float)g_vi_h;
        for (int i = 0; i < g_car_plate_box_cnt; i++) {
            const car_plate_box_t *pb = &g_car_plate_boxes[i];
            if (!pb->valid) continue;
            int px  = (int)(pb->x  * pxs);
            int py  = (int)(pb->y  * pys);
            int px2 = (int)(pb->x2 * pxs);
            int py2 = (int)(pb->y2 * pys);
            // 单层(SinglePlate=0) 绿色, 双层(DoublePlate=1) 红色, 便于排查检测效果
            RK_U32 pc = (pb->class_type == 1) ? COL_RED : COL_GREEN;
            draw_rect(buf, stride, g_main_h, px, py, px2 - px, py2 - py, pc, 2);
            if (pb->plate_number[0]) {
                int pscale = g_osd_plate_scale; if (pscale < 1) pscale = 1;
                g_have_plate_text = 1;
                char plate_label[64];
                snprintf(plate_label, sizeof(plate_label), "%s %.2f", pb->plate_number, pb->conf);
                int tw = plate_text_width(pb->plate_number, pscale);
                int lh = PLATE_H * pscale;
                // 水平钳制: 保证整个车牌号文字(px+3 .. px+3+tw)落在画布内, 避免左右超界被裁
                int tx = px + 3;
                if (tx < 0) tx = 0;
                if (tx + tw > cw) tx = (cw > tw) ? cw - tw : 0;
                // 文字优先显示在车牌框上方, 避免底部超界截断
                // 注意: 画布 chh=ALIGN16(g_main_h) 可能比实际显示高度高几行,
                //       底部边界必须用实际主码流高度 g_main_h, 否则会被显示裁剪
                int ly = py - lh - 2;
                if (ly < 0) ly = py2 + 2;          // 上方放不下才放下方
                if (ly + lh + 2 > g_main_h) ly = g_main_h - lh - 2;  // 仍超界则贴底
                if (ly < 0) ly = 0;
                if (g_debug) {
                    char hexbuf[160] = "";
                    for (int i = 0; pb->plate_number[i] && i < 48; i++)
                        sprintf(hexbuf + strlen(hexbuf), "%02X ", (unsigned char)pb->plate_number[i]);
                    printf("[PLATE_POS] label=%s px=%d py=%d px2=%d py2=%d tx=%d tw=%d ly=%d lh=%d cw=%d chh=%d stride=%d hex=%s\n",
                           plate_label, px, py, px2, py2, tx, tw, ly, lh, cw, chh, stride, hexbuf);
                }
                // 背景需覆盖文字完整范围: 文字 y 起点为 ly+2, 高度 lh, 下缘 ly+2+lh
                // 背景若只到 ly+lh, 文字底部笔画会落在黑条外被视频内容"吞掉"(视觉上像下半截断)
                for (int yy = ly; yy < ly + lh + 2 && yy < g_main_h; yy++)
                    for (int xx = tx; xx < tx + tw + 6 && xx < cw; xx++)
                        if (xx >= 0 && yy >= 0) buf[yy * stride + xx] = COL_BG;
                // 用统一 16x16 车牌字库(含中文省份字)绘制车牌号
                // 注意传入纯车牌号 pb->plate_number, 而非含空格/置信度的 plate_label
                // (空格不在字库会占位空白; 中文 UTF-8 若走 draw_text 5x7 会被当空格丢弃)
                draw_plate_text(buf, stride, g_main_h, tx, ly + 1, pb->plate_number, COL_YELLOW, pscale);
                if (g_debug) {
                    // 诊断: dump 车牌文字区域(含背景) 画布像素, 确认字形/背景是否完整
                    for (int yy = ly; yy < ly + lh + 4 && yy < g_main_h; yy++) {
                        char ln[300]; int c = 0;
                        for (int xx = tx; xx < tx + tw + 6 && xx < cw && c < 295; xx++) {
                            RK_U32 v = buf[yy * stride + xx];
                            unsigned char a = v & 0xFF;
                            unsigned char b = (v >> 24) & 0xFF, g_ = (v >> 16) & 0xFF, r = (v >> 8) & 0xFF;
                            if (r > 150 && g_ > 150 && b < 120) ln[c++] = '#';   // 黄字
                            else if (r < 80 && g_ < 80 && b > 100) ln[c++] = 'B'; // 蓝底
                            else if (a < 0x80) ln[c++] = '.';                     // 透明
                            else ln[c++] = '?';
                        }
                        ln[c] = 0;
                        printf("[CANVAS] y=%d %s\n", yy, ln);
                    }
                }
            }
        }
    }

    if (g_debug && g_have_plate_text) {
        // 只在绘制了车牌号文字的帧保存 canvas, 且文件名带 V2 标记以区分代码版本
        static int save_n = 0;
        if (save_n < 5) {
            char p[64]; snprintf(p, sizeof(p), "/tmp/canvas_V2_%d.ppm", save_n);
            save_canvas_ppm(p, buf, cw, g_main_h, stride);
            save_n++;
        }
    }

    RK_MPI_RGN_UpdateCanvas(g_rgn);
}
// ---------------- RTSP 推流线程: 取 VENC 码流 -> RTSP ----------------
static void *stream_thread(void *arg) {
    (void)arg;
    printf("stream thread start, push_stream=%d rtsp://<ip>:%d/live/0\n",
           g_push_stream, g_rtsp_port);
    VENC_STREAM_S stFrame;
    stFrame.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
    int venc_fail = 0;
    // VENC u64PTS 恒为 0(喂流设 0), 播放器遇 PTS=0 只渲染首帧不刷新。主码流已被 vdec_to_vpss_thread
    // 按 file_fps 精确限速到 25fps(VENC-OUT 验证稳定 25fps), 因此用固定每帧间隔设 PTS, 与基线
    // test_vdec_vpss_venc_rtsp.c(可正常播放)一致: 每帧 +1000000/file_fps us, 播放器按该间隔渲染。
    int64_t frame_us = 1000000LL / (g_file_fps > 0 ? g_file_fps : 25);
    uint64_t pts = 0;
    int venc_cnt = 0;                 // VENC 实际输出帧数 (诊断主码流是否被限速)
    struct timespec tsd0;
    clock_gettime(CLOCK_MONOTONIC, &tsd0);
    while (!g_quit) {
        int ret = RK_MPI_VENC_GetStream(0, &stFrame, 200000);
        if (ret == RK_SUCCESS) {
            venc_fail = 0;
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            pts += (uint64_t)frame_us;
            // 推流关闭时只排空 VENC 码流(防止缓冲积压回压 VI), 不发送到网络
            if (g_push_stream && g_rtsp && g_rtsp_sess) {
                const unsigned char *pData =
                    (const unsigned char *)RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
                rtsp_tx_video(g_rtsp_sess, pData, stFrame.pstPack->u32Len, pts);
                rtsp_do_event(g_rtsp);
            }
            RK_MPI_VENC_ReleaseStream(0, &stFrame);
            venc_cnt++;
            double d = ((double)ts.tv_sec - tsd0.tv_sec) +
                       ((double)ts.tv_nsec - tsd0.tv_nsec) / 1e9;
            if (d >= 5.0) {
                printf("[VENC-OUT] %.1f fps over %.1fs (%d frames)\n", venc_cnt / d, d, venc_cnt);
                venc_cnt = 0; tsd0 = ts;
            }
        } else {
            if (++venc_fail % 100 == 1)
                printf("stream: VENC_GetStream fail ret=%x fail=%d\n", ret, venc_fail);
        }
    }
    free(stFrame.pstPack);
    return NULL;
}

// ---------------- 三段式 AI 流水线: 预处理/推理(NPU)/后处理 三线程重叠 ----------------
// 用槽位池 + 3 条队列(空闲/待推理/待后处理)连接 3 个线程。
// 帧率上限 ≈ max(rknn_run, 预处理, 后处理) ≈ rknn_run(~50ms) -> 接近 20fps。
#define PIPE_SLOTS     3
#define PIPE_MAX_OUT   16

typedef struct {
    image_buffer_t dst_img;      // 640x640 RGB letterbox 结果 (持久分配)
    image_buffer_t full_rgb;     // 整幅 chn1 RGB (用于 DeepSORT ReID 裁剪, 持久分配)
    letterbox_t letter_box;
    rknn_output outputs[PIPE_MAX_OUT];
    object_detect_result_list od;
    int seq;
    double infer_ms;       // 检测模型 NPU 推理耗时 (infer_thread 写入)
    double track_ms;       // 跟踪耗时 (post_thread 写入)
} pipe_slot_t;

static pipe_slot_t g_slots[PIPE_SLOTS];

typedef struct {
    int idx[PIPE_SLOTS];
    int head;
    int count;
} pipe_queue_t;

static pipe_queue_t g_free_q, g_pre_q, g_infer_q;   // 空闲 -> 待推理 -> 待后处理 -> 空闲
static pthread_mutex_t g_free_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_pre_lock   = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_infer_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_free_cond  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_pre_cond   = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_infer_cond = PTHREAD_COND_INITIALIZER;

static void pipe_q_push(pipe_queue_t *q, int s) { q->idx[(q->head + q->count) % PIPE_SLOTS] = s; q->count++; }
static int  pipe_q_pop(pipe_queue_t *q) { int s = q->idx[q->head]; q->head = (q->head + 1) % PIPE_SLOTS; q->count--; return s; }

// 阶段1: 预处理线程 - VI取帧 -> NV12->RGB -> letterbox -> 交给推理线程
static void *preprocess_thread(void *arg) {
    (void)arg;
    printf("preprocess thread start\n");
    image_buffer_t yuv;
    memset(&yuv, 0, sizeof(yuv));

    int fail_cnt = 0;
    int fail_log = 0;
    while (!g_quit) {
        // 取空闲槽
        pthread_mutex_lock(&g_free_lock);
        while (g_free_q.count == 0 && !g_quit) pthread_cond_wait(&g_free_cond, &g_free_lock);
        if (g_quit) { pthread_mutex_unlock(&g_free_lock); break; }
        int s = pipe_q_pop(&g_free_q);
        pthread_mutex_unlock(&g_free_lock);

        VIDEO_FRAME_INFO_S frame;
        memset(&frame, 0, sizeof(frame));
        int ret = RK_MPI_VPSS_GetChnFrame(0, 1, &frame, 1000);
        if (ret != RK_SUCCESS) {
            if (++fail_cnt % 30 == 1)
                printf("[PRE] get frame fail ret=%x fail=%d\n", ret, fail_cnt);
            // 取帧失败: 归还槽位给空闲池
            pthread_mutex_lock(&g_free_lock);
            pipe_q_push(&g_free_q, s);
            pthread_mutex_unlock(&g_free_lock);
            usleep(10000);
            continue;
        }
        fail_cnt = 0;

        void *vaddr = RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
        if (vaddr) {
            if (fail_log < 6) {
                unsigned char *p = (unsigned char *)vaddr;
                int nz = 0;
                for (int i = 0; i < 4096; i++) if (p[i]) nz++;
                printf("[PRE] chn1 fmt=%d %ux%u vir=%ux%u nz=%d/%d bytes=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                       (int)frame.stVFrame.enPixelFormat,
                       (unsigned)frame.stVFrame.u32Width, (unsigned)frame.stVFrame.u32Height,
                       (unsigned)frame.stVFrame.u32VirWidth, (unsigned)frame.stVFrame.u32VirHeight,
                       nz, 4096,
                       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
                fail_log++;
            }
            yuv.virt_addr = (unsigned char *)vaddr;
            yuv.width = frame.stVFrame.u32Width;
            yuv.height = frame.stVFrame.u32Height;
            yuv.width_stride = frame.stVFrame.u32VirWidth;
            yuv.format = IMAGE_FORMAT_YUV420SP_NV12;
            get_image_size(&yuv);

            image_buffer_t *frgb = &g_slots[s].full_rgb;
            frgb->width = yuv.width;
            frgb->height = yuv.height;
            convert_yuv420sp_to_rgb(&yuv, frgb);

            // letterbox RGB(AI帧) -> 640x640, 结果写入槽位(槽位 dst_img 复用已分配内存)
            yolov8_preprocess(&g_yolo, frgb, &g_slots[s].dst_img, &g_slots[s].letter_box);
            g_slots[s].seq++;

            // 交给推理线程
            pthread_mutex_lock(&g_pre_lock);
            pipe_q_push(&g_pre_q, s);
            pthread_cond_signal(&g_pre_cond);
            pthread_mutex_unlock(&g_pre_lock);
        }
        RK_MPI_VPSS_ReleaseChnFrame(0, 1, &frame);
    }
    return NULL;
}

// 阶段2: 推理线程 - inputs_set + rknn_run(NPU) + outputs_get -> 交给后处理线程
static void *infer_thread(void *arg) {
    (void)arg;
    printf("infer thread start (model %dx%d)\n", g_yolo.model_width, g_yolo.model_height);
    while (!g_quit) {
        pthread_mutex_lock(&g_pre_lock);
        while (g_pre_q.count == 0 && !g_quit) pthread_cond_wait(&g_pre_cond, &g_pre_lock);
        if (g_quit) { pthread_mutex_unlock(&g_pre_lock); break; }
        int s = pipe_q_pop(&g_pre_q);
        pthread_mutex_unlock(&g_pre_lock);

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int rk_ret = yolov8_infer(&g_yolo, &g_slots[s].dst_img, g_slots[s].outputs);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        g_slots[s].infer_ms = ((double)(t1.tv_sec - t0.tv_sec) * 1000.0 +
                               (double)(t1.tv_nsec - t0.tv_nsec) / 1e6);

        if (rk_ret == 0) {
            pthread_mutex_lock(&g_infer_lock);
            pipe_q_push(&g_infer_q, s);
            pthread_cond_signal(&g_infer_cond);
            pthread_mutex_unlock(&g_infer_lock);
        } else {
            // 推理失败: 释放槽位回空闲池
            pthread_mutex_lock(&g_free_lock);
            pipe_q_push(&g_free_q, s);
            pthread_mutex_unlock(&g_free_lock);
        }
    }
    return NULL;
}

// ByteTrack: 用最新检测结果更新跟踪器, 把跟踪ID写回 od 结果 (-1=未跟踪)
static void byteTrack_update(object_detect_result_list *od) {
    std::vector<Object> objects;
    objects.reserve(od->count);
    for (int i = 0; i < od->count; i++) {
        const object_detect_result *r = &od->results[i];
        Object obj;
        obj.rect.x = r->box.left;
        obj.rect.y = r->box.top;
        obj.rect.width  = r->box.right  - r->box.left;
        obj.rect.height = r->box.bottom - r->box.top;
        obj.label = r->cls_id;
        obj.prob  = r->prop;
        objects.push_back(obj);
    }

    std::vector<STrack> tracks = g_tracker.update(objects);

    for (int i = 0; i < od->count; i++) od->results[i].track_id = -1;

    // 用 IoU 把跟踪结果匹配回检测框 (ByteTrack 内部会重排结果)
    for (size_t t = 0; t < tracks.size(); t++) {
        const STrack &tr = tracks[t];
        float tx = tr.tlwh[0], ty = tr.tlwh[1];
        float tw = tr.tlwh[2], th = tr.tlwh[3];
        int best = -1; float best_iou = 0;
        for (int i = 0; i < od->count; i++) {
            const object_detect_result *r = &od->results[i];
            float ix = r->box.left, iy = r->box.top;
            float iw = r->box.right - r->box.left, ih = r->box.bottom - r->box.top;
            float x1 = tx > ix ? tx : ix, y1 = ty > iy ? ty : iy;
            float x2 = (tx + tw) < (ix + iw) ? (tx + tw) : (ix + iw);
            float y2 = (ty + th) < (iy + ih) ? (ty + th) : (iy + ih);
            float inter = (x2 - x1 > 0 ? x2 - x1 : 0) * (y2 - y1 > 0 ? y2 - y1 : 0);
            float uni = tw * th + iw * ih - inter;
            if (uni <= 0) continue;
            float iou = inter / uni;
            if (iou > best_iou) { best_iou = iou; best = i; }
        }
        if (best >= 0 && best_iou > 0.1f)
            od->results[best].track_id = tr.track_id;
    }
}

// 统一跟踪入口: 按 config 选择 ByteTrack / DeepSORT
// full_rgb 为该帧 chn1 RGB (仅 DeepSORT 需要, 用于 ReID 裁剪)
static void track_update(object_detect_result_list *od, image_buffer_t *full_rgb) {
    if (g_tracker_type == 1) {
        deepsort_map_t map;
        map.ai_w = (int)g_ai_w;
        map.ai_h = (int)g_ai_h;
        map.roi_ai_x0 = g_main_roi_ai_x0;
        map.roi_ai_y0 = g_main_roi_ai_y0;
        map.roi_ai_sx = g_main_roi_ai_sx;
        map.roi_ai_sy = g_main_roi_ai_sy;
        map.full_w = (int)g_chn_w;
        map.full_h = (int)g_chn_h;
        deepsort_update(full_rgb, od, &map);
    } else {
        byteTrack_update(od);
    }
}

// 阶段3: 后处理线程 - post_process -> 刷新RGN/统计 -> release outputs -> 归还空闲池
// 车牌色中文化 (TCP 输出用; 原 plate_color_cn 为 car_integ 内部 static)
static const char *plate_color_cn_tcp(const char *en) {
    if (!strcmp(en, "Black")) return "黑色";
    if (!strcmp(en, "Blue"))  return "蓝色";
    if (!strcmp(en, "Green")) return "绿色";
    if (!strcmp(en, "White")) return "白色";
    if (!strcmp(en, "Yellow"))return "黄色";
    return en;
}

// TCP 输出: 组装并发送每帧的 frame (跟踪) + detection (原始识别) NDJSON 消息。
// box 均以 0-1 归一化 (整幅-AI: frame 用 g_ai_w/g_ai_h; detection 用整幅 fw/fh)。
static void tcp_send_detections(const object_detect_result_list *od)
{
    if (!g_tcp_cfg.enable) return;

    double ts = 0.0;
    struct timespec tv;
    if (clock_gettime(CLOCK_REALTIME, &tv) == 0)
        ts = (double)tv.tv_sec + (double)tv.tv_nsec / 1e9;
    long seq = ++g_tcp_frame_seq;

    // 1) frame: 每帧跟踪结果, 全类别, cls=数字, box 0-1 (整幅-AI 基准)
    {
        char line[8192];
        int off = snprintf(line, sizeof(line),
                           "{\"type\":\"frame\",\"seq\":%ld,\"ts\":%.3f,\"objs\":[",
                           seq, ts);
        for (int i = 0; i < od->count && off < (int)sizeof(line) - 64; i++) {
            const object_detect_result *r = &od->results[i];
            float x0 = (g_ai_w > 0) ? (float)r->box.left   / g_ai_w : 0.0f;
            float y0 = (g_ai_h > 0) ? (float)r->box.top    / g_ai_h : 0.0f;
            float x1 = (g_ai_w > 0) ? (float)r->box.right  / g_ai_w : 0.0f;
            float y1 = (g_ai_h > 0) ? (float)r->box.bottom / g_ai_h : 0.0f;
            if (i > 0) { if (off >= (int)sizeof(line) - 64) break; off += snprintf(line + off, sizeof(line) - off, ","); }
            off += snprintf(line + off, sizeof(line) - off,
                            "{\"cls\":%d,\"conf\":%.3f,\"track_id\":%d,\"box\":[%.4f,%.4f,%.4f,%.4f]}",
                            r->cls_id, r->prop, r->track_id, x0, y0, x1, y1);
        }
        if (off < (int)sizeof(line) - 4)
            snprintf(line + off, sizeof(line) - off, "]}");
        tcp_output_send_line(line);
    }

    // 2) detection: 最近一次车牌/车辆原始识别结果 (final_result_t)
    if (g_car_cfg.enable) {
        std::vector<final_result_t> raw;
        int fw = 0, fh = 0; long rseq = 0;
        if (car_integ_get_last_raw(&raw, &fw, &fh, &rseq) == 1 && !raw.empty()) {
            char line[16384];
            int off = snprintf(line, sizeof(line),
                               "{\"type\":\"detection\",\"ts\":%.3f,\"fw\":%d,\"fh\":%d,\"objs\":[",
                               ts, fw, fh);
            for (size_t i = 0; i < raw.size() && off < (int)sizeof(line) - 96; i++) {
                const final_result_t &r = raw[i];
                // rect 为 ROI 相对坐标, 加 roi_x/y 映射回整幅, 再 /fw,/fh 归一化
                float px = (float)(g_car_cfg.roi_x + r.rect.x);
                float py = (float)(g_car_cfg.roi_y + r.rect.y);
                float x0 = (fw > 0) ? px / (float)fw : 0.0f;
                float y0 = (fh > 0) ? py / (float)fh : 0.0f;
                float x1 = (fw > 0) ? (px + r.rect.width)  / (float)fw : 0.0f;
                float y1 = (fh > 0) ? (py + r.rect.height) / (float)fh : 0.0f;
                char pn[96], pc[48], cc[48];
                tcp_output_json_escape(r.plate_number.c_str(), pn, sizeof(pn));
                tcp_output_json_escape(plate_color_cn_tcp(r.plate_color.c_str()), pc, sizeof(pc));
                tcp_output_json_escape(r.car_color.c_str(), cc, sizeof(cc));
                if (i > 0) { if (off >= (int)sizeof(line) - 96) break; off += snprintf(line + off, sizeof(line) - off, ","); }
                off += snprintf(line + off, sizeof(line) - off,
                                "{\"class\":%d,\"conf\":%.3f,\"plate\":\"%s\",\"plate_color\":\"%s\",\"car_color\":\"%s\",\"box\":[%.4f,%.4f,%.4f,%.4f]}",
                                r.class_type, r.confidence, pn, pc, cc, x0, y0, x1, y1);
            }
            if (off < (int)sizeof(line) - 4)
                snprintf(line + off, sizeof(line) - off, "]}");
            tcp_output_send_line(line);
        }
    }
}

static void *post_thread(void *arg) {
    (void)arg;
    printf("post process thread start\n");
    int frame_cnt = 0, fps_cnt = 0;
    double sum_infer_ms = 0.0, sum_track_ms = 0.0;
    struct timespec t_now, t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);

    while (!g_quit) {
        pthread_mutex_lock(&g_infer_lock);
        while (g_infer_q.count == 0 && !g_quit) pthread_cond_wait(&g_infer_cond, &g_infer_lock);
        if (g_quit) { pthread_mutex_unlock(&g_infer_lock); break; }
        int s = pipe_q_pop(&g_infer_q);
        pthread_mutex_unlock(&g_infer_lock);

        pipe_slot_t *sl = &g_slots[s];
        if (yolov8_postprocess(&g_yolo, sl->outputs, &sl->letter_box, &sl->od) == 0) {
            object_detect_result_list *od = &sl->od;

            // 主线程 ROI 反偏移: 检测结果原为"ROI-AI 坐标"(chn1 裁剪后的放大区域),
            // 回归到"整幅-AI 坐标"(0..ai_w x 0..ai_h 代表整幅源图), 供 ByteTrack/车牌关联/OSD 统一使用。
            if (g_main_roi_enable) {
                for (int i = 0; i < od->count; i++) {
                    object_detect_result *r = &od->results[i];
                    r->box.left   = (int)(g_main_roi_ai_x0 + r->box.left   * g_main_roi_ai_sx);
                    r->box.right  = (int)(g_main_roi_ai_x0 + r->box.right  * g_main_roi_ai_sx);
                    r->box.top    = (int)(g_main_roi_ai_y0 + r->box.top    * g_main_roi_ai_sy);
                    r->box.bottom = (int)(g_main_roi_ai_y0 + r->box.bottom * g_main_roi_ai_sy);
                }
            }

            struct timespec tt0, tt1;
            clock_gettime(CLOCK_MONOTONIC, &tt0);
            track_update(od, &sl->full_rgb);
            clock_gettime(CLOCK_MONOTONIC, &tt1);
            sl->track_ms = ((double)(tt1.tv_sec - tt0.tv_sec) * 1000.0 +
                            (double)(tt1.tv_nsec - tt0.tv_nsec) / 1e6);

            // 车牌/车辆识别: 每帧调用 (内部非阻塞拉取 worker 结果 + 每帧回填已采信属性)
            if (g_car_cfg.enable) {
                car_integ_run(od);
            }

            pthread_mutex_lock(&g_od_lock);
            g_od = *od;
            pthread_mutex_unlock(&g_od_lock);
            rgn_update(od);

            // TCP + NDJSON 输出 (frame/detection)
            tcp_send_detections(od);

            for (int k = 0; k < od->count; k++) {
                const object_detect_result *r = &od->results[k];
                if (g_debug) {
                    printf("[DET] %-12s conf=%.2f id=%d box=(%d,%d,%d,%d)\n",
                           cls_to_name(r->cls_id), r->prop, r->track_id,
                           r->box.left, r->box.top, r->box.right, r->box.bottom);
                } else if (r->has_plate && r->plate_number[0] &&
                           plate_print_gate(r->track_id, r->plate_number)) {
                    // 非调试模式: 只打印检测到的车牌 (车牌号+车牌色+车身色), 限速防刷屏
                    printf("[PLATE] id=%d %s %s", r->track_id, r->plate_number, r->plate_color);
                    if (r->car_color[0]) printf(" 车色:%s", r->car_color);
                    printf("\n");
                }
            }

            fps_cnt++;
            sum_infer_ms += sl->infer_ms;
            sum_track_ms += sl->track_ms;
            clock_gettime(CLOCK_MONOTONIC, &t_now);
            double dt = (t_now.tv_sec - t_last.tv_sec) +
                        (t_now.tv_nsec - t_last.tv_nsec) / 1e9;
            if (dt >= (double)g_print_interval_s) {
                g_fps = fps_cnt / dt;
                printf("[FPS] %.1f fps  objs=%d | infer=%.2fms track=%.2fms car=%.1fms (avg over %.1fs, %d frames)\n",
                       g_fps, od->count,
                       (fps_cnt ? sum_infer_ms / fps_cnt : 0.0),
                       (fps_cnt ? sum_track_ms / fps_cnt : 0.0),
                       car_integ_get_last_ms(), dt, fps_cnt);
                fps_cnt = 0;
                sum_infer_ms = 0.0; sum_track_ms = 0.0;
                t_last = t_now;
            }
            frame_cnt++;
        }

        // 后处理完成, 释放 rknn 输出缓冲并归还槽位给空闲池
        rknn_outputs_release(g_yolo.rknn_ctx, g_yolo.io_num.n_output, sl->outputs);
        pthread_mutex_lock(&g_free_lock);
        pipe_q_push(&g_free_q, s);
        pthread_cond_signal(&g_free_cond);
        pthread_mutex_unlock(&g_free_lock);
    }
    return NULL;
}

// 启动三段流水线 (替代旧串行 ai_thread)
static void ai_pipeline_init(void) {
    for (int i = 0; i < PIPE_SLOTS; i++) {
        memset(&g_slots[i], 0, sizeof(g_slots[i]));
        g_slots[i].dst_img.width = g_yolo.model_width;
        g_slots[i].dst_img.height = g_yolo.model_height;
        g_slots[i].dst_img.format = IMAGE_FORMAT_RGB888;
        g_slots[i].dst_img.size = get_image_size(&g_slots[i].dst_img);
        g_slots[i].dst_img.virt_addr = (unsigned char *)malloc(g_slots[i].dst_img.size);
        if (!g_slots[i].dst_img.virt_addr)
            printf("[PIPE] malloc slot %d dst_img fail\n", i);
        // full_rgb: chn1 RGB888 (w*h*3), 供 DeepSORT ReID 裁剪 (每槽约 w*h*3 字节)
        g_slots[i].full_rgb.width = g_chn_w;
        g_slots[i].full_rgb.height = g_chn_h;
        g_slots[i].full_rgb.format = IMAGE_FORMAT_RGB888;
        g_slots[i].full_rgb.size = get_image_size(&g_slots[i].full_rgb);
        g_slots[i].full_rgb.virt_addr = (unsigned char *)malloc(g_slots[i].full_rgb.size);
        if (!g_slots[i].full_rgb.virt_addr)
            printf("[PIPE] malloc slot %d full_rgb fail\n", i);
    }
    memset(&g_free_q, 0, sizeof(g_free_q));
    memset(&g_pre_q, 0, sizeof(g_pre_q));
    memset(&g_infer_q, 0, sizeof(g_infer_q));
    for (int i = 0; i < PIPE_SLOTS; i++) pipe_q_push(&g_free_q, i);
    printf("AI 3-stage pipeline ready (slots=%d)\n", PIPE_SLOTS);
}

static void ai_pipeline_cleanup(void) {
    for (int i = 0; i < PIPE_SLOTS; i++) {
        if (g_slots[i].dst_img.virt_addr) free(g_slots[i].dst_img.virt_addr);
        if (g_slots[i].full_rgb.virt_addr) free(g_slots[i].full_rgb.virt_addr);
    }
}

// ---------------- 统一资源释放(正常退出与失败路径共用, 尽量安全) ----------------
static void media_cleanup(void) {
    if (g_inited_rgn) {
        MPP_CHN_S mpp = { RK_ID_VENC, 0, 0 };
        RK_MPI_RGN_DetachFromChn(g_rgn, &mpp);
        RK_MPI_RGN_Destroy(g_rgn);
        g_inited_rgn = 0;
    }
    if (g_inited_venc) {
        RK_MPI_VENC_StopRecvFrame(0);
        RK_MPI_VENC_DestroyChn(0);
        g_inited_venc = 0;
    }
    if (g_bound_main) {
        // VPSS chn0 -> VENC (两模式共用)
        MPP_CHN_S src = { RK_ID_VPSS, 0, 0 }, dst = { RK_ID_VENC, 0, 0 };
        RK_MPI_SYS_UnBind(&src, &dst);
        g_bound_main = 0;
    }
    if (g_inited_vpss) {
        RK_MPI_VPSS_StopGrp(0);
        RK_MPI_VPSS_DisableChn(0, 0);
        RK_MPI_VPSS_DisableChn(0, 1);
        if (g_car_cfg.enable && g_car_cfg.chn != 0 && g_car_cfg.chn != 1)
            RK_MPI_VPSS_DisableChn(0, g_car_cfg.chn);
        RK_MPI_VPSS_DestroyGrp(0);
        g_inited_vpss = 0;
    }
    if (g_bound_vdec) {
        // 源 -> VPSS: VI(摄像头) / VDEC(文件)
        MPP_CHN_S src = { g_file_mode ? RK_ID_VDEC : RK_ID_VI, 0, 0 }, grp = { RK_ID_VPSS, 0, 0 };
        RK_MPI_SYS_UnBind(&src, &grp);
        g_bound_vdec = 0;
    }
    if (g_file_mode) {
        if (g_inited_vdec) {
            RK_MPI_VDEC_StopRecvStream(0);
            RK_MPI_VDEC_DestroyChn(0);
            g_inited_vdec = 0;
        }
        if (g_vdec_pool != MB_INVALID_POOLID) {
            RK_MPI_MB_DestroyPool(g_vdec_pool);
            g_vdec_pool = MB_INVALID_POOLID;
        }
    }
    if (g_inited_vi) {
        RK_MPI_VI_DisableChn(0, 0);
        RK_MPI_VI_DisableDev(0);
        g_inited_vi = 0;
    }
    if (g_inited_sys) { RK_MPI_SYS_Exit(); g_inited_sys = 0; }
}

// ---------------- main ----------------
static void usage(const char *p) {
    printf("Usage: %s [-c config] [-m model] [-a iqdir] [-l labels] [-w W] [-h H]\n", p);
    printf("          [-e h264|h265] [-p port] [-P path] [-T conf] [-N nms] [-C classnum]\n");
    printf("  config file keys: model iq_dir labels width height ai_w bitrate port path\n");
    printf("                     codec conf_thresh nms_thresh class_num show_osd show_fps debug\n");
    printf("                     car_enable car_src_mode car_roi_x car_roi_y car_roi_w car_roi_h\n");
    printf("                     car_chn car_interval_ms car_plate_type car_detect_model car_plate_model car_car_model\n");
}

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);   // 无缓冲, 重定向到文件也能实时看到日志
    // 车牌/车辆识别默认配置 (可被配置文件覆盖)
    g_car_cfg.enable      = 0;
    g_car_cfg.debug       = 0;
    g_car_cfg.src_mode    = -1;         // <0=自动按模式推断 (摄像头VI/文件VPSS)
    g_car_cfg.roi_x = 0; g_car_cfg.roi_y = 0;
    g_car_cfg.roi_w = 0; g_car_cfg.roi_h = 0;
    g_car_cfg.roi_nx = 0.0f; g_car_cfg.roi_ny = 0.0f;
    g_car_cfg.roi_nw = 0.0f; g_car_cfg.roi_nh = 0.0f;
    g_car_cfg.chn         = 2;          // 第三路通道
    g_car_cfg.interval_ms = 1000;
    g_car_cfg.plate_type  = 0;          // 默认 plate_rec; 设 1/LPRNet 切换
    g_car_cfg.detect_conf_thresh = 0.3f;
    g_car_cfg.detect_nms_thresh  = 0.5f;
    g_car_cfg.plate_nms_thresh   = 0.1f;
    g_car_cfg.plate_vote_thresh  = 3;
    g_car_cfg.car_color_vote_thresh = 2;
    g_car_cfg.max_track_age_frames = 90;
    g_car_cfg.match_iou_thresh   = 0.1f;
    snprintf(g_car_cfg.detect_model, sizeof(g_car_cfg.detect_model),
             "/userdata/car_demo/detect.rknn");
    snprintf(g_car_cfg.plate_model, sizeof(g_car_cfg.plate_model),
             "/userdata/car_demo/plate_rec_color.rknn");
    snprintf(g_car_cfg.car_model, sizeof(g_car_cfg.car_model),
             "/userdata/car_demo/car_recognition.rknn");

    // 第一遍: 仅找 -c 配置文件, 先加载(默认路径兜底)
    const char *cfg_path = NULL;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "-c") && i + 1 < argc) cfg_path = argv[i + 1];
    load_config(cfg_path ? cfg_path : "/userdata/yolo/yolo.conf");
    g_car_cfg.debug = g_debug;   // 同步 debug 开关到车牌模块

    // 第二遍: 解析其余命令行参数, 覆盖配置文件
    int c;
    while ((c = getopt(argc, argv, "c:m:a:l:w:h:e:p:P:T:N:C:v:")) != -1) {
        switch (c) {
        case 'c': break;  // 已处理
        case 'v': snprintf(g_video_file, sizeof(g_video_file), "%s", optarg); g_file_mode = 1; break;
        case 'm': snprintf(g_model_path, sizeof(g_model_path), "%s", optarg); break;
        case 'a': snprintf(g_iq_dir, sizeof(g_iq_dir), "%s", optarg); break;
        case 'l': snprintf(g_label_path, sizeof(g_label_path), "%s", optarg); break;
        case 'w': g_main_w = atoi(optarg); break;
        case 'h': g_main_h = atoi(optarg); break;
        case 'e': g_codec = (!strcmp(optarg, "h265")) ? RK_VIDEO_ID_HEVC : RK_VIDEO_ID_AVC; break;
        case 'p': g_rtsp_port = atoi(optarg); break;
        case 'P': snprintf(g_rtsp_path, sizeof(g_rtsp_path), "%s", optarg); break;
        case 'T': g_box_conf_thresh = (float)atof(optarg); break;
        case 'N': g_nms_thresh = (float)atof(optarg); break;
        case 'C': g_obj_class_num = atoi(optarg); break;
        default: usage(argv[0]); return -1;
        }
    }

    // AI 流按主码流 16:9 等比缩小
    g_ai_h = g_ai_w * g_main_h / g_main_w;
    if (g_file_mode) {
        // 文件模式: 未单独指定解码分辨率时, 用主码流尺寸作为解码源
        g_dec_w = g_main_w; g_dec_h = g_main_h;
    }
    // 车牌识别帧源: 统一走 VPSS 通道 (src_mode 保留字段, 不再区分 VI/VPSS)
    g_car_cfg.ai_w = g_ai_w; g_car_cfg.ai_h = g_ai_h;  // 供坐标映射
    printf("# yolo_rtsp v%s build " __DATE__ " " __TIME__ "\n", APP_VERSION);
    printf("# mode=%s main %dx%d ai %dx%d chn %dx%d codec=%s port=%d path=%s conf=%.2f nms=%.2f push_stream=%d\n",
           g_file_mode ? "FILE" : "CAMERA",
           g_main_w, g_main_h, g_ai_w, g_ai_h, g_chn_w, g_chn_h,
           g_codec == RK_VIDEO_ID_HEVC ? "H265" : "H264", g_rtsp_port, g_rtsp_path,
           g_box_conf_thresh, g_nms_thresh, g_push_stream);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    // 0. 标签(类别名)
    printf("cfg: model=[%s] labels=[%s] video=[%s] file_mode=%d\n",
           g_model_path, g_label_path, g_video_file, g_file_mode);
    printf("osd: scale=%d color=0x%08X\n", g_osd_scale, g_osd_color);
    init_post_process(g_label_path);

    // 1. ISP(仅摄像头模式)
    if (!g_file_mode && isp_init(0, g_iq_dir) != 0) return -1;

    // 2. RTSP
    g_rtsp = create_rtsp_demo(g_rtsp_port);
    g_rtsp_sess = rtsp_new_session(g_rtsp, g_rtsp_path);
    if (!g_rtsp || !g_rtsp_sess) {
        printf("RTSP init fail (port=%d path='%s')\n", g_rtsp_port, g_rtsp_path);
        goto fail;
    }
    printf("RTSP ready: rtsp://<ip>:%d%s\n", g_rtsp_port, g_rtsp_path);
    if (g_codec == RK_VIDEO_ID_AVC)
        rtsp_set_video(g_rtsp_sess, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    else
        rtsp_set_video(g_rtsp_sess, RTSP_CODEC_ID_VIDEO_H265, NULL, 0);
    rtsp_sync_video_ts(g_rtsp, rtsp_get_reltime(), rtsp_get_ntptime());

    // 3. 媒体系统
    if (RK_MPI_SYS_Init() != RK_SUCCESS) { printf("SYS_Init fail\n"); goto fail; }
    g_inited_sys = 1;

    // 4. 视频通路
    if (g_file_mode) {
        // 文件模式: VDEC -> VPSS(主码流+AI流+车牌), VENC/RTSP 复用
        if (file_pipeline_init() != 0) goto fail;
    } else {
        // 摄像头模式: VI(源分辨率 4K) -> VPSS(主码流+AI流+车牌)
        if (camera_pipeline_init() != 0) goto fail;
    }

    // 5. VENC
    if (venc_init(0, g_main_w, g_main_h) != 0) goto fail;
    g_inited_venc = 1;

    // 6. RGN 画框
    if (rgn_init() != 0) goto fail;
    g_inited_rgn = 1;

    // 7. 主码流 -> VENC0
    //    VI(或VDEC) -> VPSS chn0 -> VENC
    printf("stage: binding main->venc...\n"); fflush(stdout);
    if (g_file_mode) {
        // 文件模式: VDEC->VPSS 改用手动分发线程 (vdec_to_vpss_thread) 投递解码帧,
        // 不在此硬件绑定 (RV1126 绑定后 VPSS 不从 VDEC 拉流, chn1 无帧导致 AI 无输入)。
        // VDEC 帧缓冲在 file_pipeline_init 中通过 MB_SOURCE_USER + MB Pool 提供。
        g_bound_vdec = 0;
    } else {
        // VI chn0(源分辨率) -> VPSS 硬件绑定
        MPP_CHN_S vi0 = { RK_ID_VI, 0, 0 }, grp = { RK_ID_VPSS, 0, 0 };
        if (RK_MPI_SYS_Bind(&vi0, &grp) != RK_SUCCESS) {
            printf("bind VI->VPSS fail\n"); goto fail;
        }
        g_bound_vdec = 1;
    }
    {
        // VPSS chn0 -> VENC (两模式共用)
        MPP_CHN_S src = { RK_ID_VPSS, 0, 0 }, dst = { RK_ID_VENC, 0, 0 };
        if (RK_MPI_SYS_Bind(&src, &dst) != RK_SUCCESS) {
            printf("bind VPSS->VENC fail\n"); goto fail;
        }
    }
    g_bound_main = 1;
    printf("stage: bind ok. loading model...\n"); fflush(stdout);

    // 8. 加载 YOLO 模型(类别数自动从模型输出推断)
    memset(&g_yolo, 0, sizeof(g_yolo));
    if (init_yolov8_model(g_model_path, &g_yolo) != 0) {
        printf("init_yolov8_model fail: %s\n", g_model_path);
        goto fail;
    }

    // 8.5 车牌/车辆识别集成初始化 (三模型管线)
    // 归一化 car_roi (0-1, 相对源帧) -> 源帧像素, 供 car_lib 内部裁剪/偏移使用
    {
        RK_U32 src_w = g_file_mode ? g_dec_w : g_vi_w;
        RK_U32 src_h = g_file_mode ? g_dec_h : g_vi_h;
        g_car_cfg.roi_x = (int)(g_car_cfg.roi_nx * src_w);
        g_car_cfg.roi_y = (int)(g_car_cfg.roi_ny * src_h);
        g_car_cfg.roi_w = (int)(g_car_cfg.roi_nw * src_w);
        g_car_cfg.roi_h = (int)(g_car_cfg.roi_nh * src_h);
        printf("car roi: norm=(%.3f,%.3f %.3fx%.3f) -> px=(%d,%d %dx%d) src=%dx%d\n",
               g_car_cfg.roi_nx, g_car_cfg.roi_ny, g_car_cfg.roi_nw, g_car_cfg.roi_nh,
               g_car_cfg.roi_x, g_car_cfg.roi_y, g_car_cfg.roi_w, g_car_cfg.roi_h, src_w, src_h);
    }
    printf("plate: show_plate=%d save_plate_image=%d dir=[%s] interval=%dms\n",
           g_show_plate, g_car_cfg.save_plate_image, g_car_cfg.save_dir, g_car_cfg.save_interval_ms);
    if (g_car_cfg.enable && car_integ_init(&g_car_cfg) != 0) {
        printf("car_integ_init fail (disable car recognition)\n");
        g_car_cfg.enable = 0;
    }
    // TCP + NDJSON 输出 (失败不阻塞主流程, 仅告警)
    if (g_tcp_cfg.enable && tcp_output_init(&g_tcp_cfg) != 0)
        printf("[TCP] init fail (disable tcp output)\n");

    // 9. 启动三段流水线线程 + 业务线程
    ai_pipeline_init();

    // DeepSORT ReID worker (仅 tracker=deepsort 时加载模型并启动; 失败回退 ByteTrack)
    if (g_tracker_type == 1) {
        if (deepsort_init(g_reid_model, g_reid_dim, g_reid_w, g_reid_h, g_reid_interval) != 0) {
            printf("[DEEPSORT] init fail, fallback to bytetrack\n");
            g_tracker_type = 0;
        }
    }

    pthread_t t_stream, t_pre, t_infer, t_post, t_feed, t_v2v;
    if (g_file_mode) {
        pthread_create(&t_feed, NULL, file_feed_thread, NULL);
        pthread_create(&t_v2v, NULL, vdec_to_vpss_thread, NULL);
    }
    pthread_create(&t_pre, NULL, preprocess_thread, NULL);
    pthread_create(&t_infer, NULL, infer_thread, NULL);
    pthread_create(&t_post, NULL, post_thread, NULL);
    pthread_create(&t_stream, NULL, stream_thread, NULL);

    while (!g_quit) usleep(500000);

    printf("exiting... stopping threads\n");
    pthread_join(t_stream, NULL);
    pthread_join(t_pre, NULL);
    pthread_join(t_infer, NULL);
    pthread_join(t_post, NULL);
    if (g_file_mode) { pthread_join(t_feed, NULL); pthread_join(t_v2v, NULL); }

    printf("releasing media...\n");
    ai_pipeline_cleanup();
    release_yolov8_model(&g_yolo);
    deepsort_deinit();
    car_integ_deinit();
    tcp_output_deinit();
    media_cleanup();
    deinit_post_process();
    if (g_rtsp) rtsp_del_demo(g_rtsp);
    isp_deinit(0);
    free(g_overlay);
    printf("bye.\n");
    return 0;

fail:
    media_cleanup();
    if (g_rtsp) rtsp_del_demo(g_rtsp);
    isp_deinit(0);
    return -1;
}
