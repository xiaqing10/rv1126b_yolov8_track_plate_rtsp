/*
 * test_vdec_h264_rtsp.c
 * 最小基线: 读 h264/h265 裸流文件 -> VDEC(STREAM 模式, 官方 MB_SOURCE_USER 帧池)
 *          -> VPSS chn0(源分辨率) -> VENC -> RTSP 推流
 *
 * 完全复刻官方 sample_comm_vdec.c 的 SAMPLE_COMM_VDEC_CreateChn / SendStream,
 * 仅把 VPSS/VENC 接到 RTSP(官方 demo 接 VO/MIPI, 我们没有屏)。
 *
 * 用法:
 *   test_vdec_h264_rtsp -f /userdata/xxx.h264 -e h264 -w 1920 -h 1080 -p 8554 -n 0
 *   test_vdec_h264_rtsp -f /userdata/xxx.h265 -e h265 -w 1920 -h 1080 -p 8554 -n 0
 *
 *   -n 0: 用 RGA 做 VPSS 缩放(VIDEO_PROC_DEV_RGA, 官方默认)
 *   -n 1: 用 VPSS 硬件(VIDEO_PROC_DEV_VPSS)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include "rk_mpi_mb.h"
#include "rk_mpi_cal.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_vdec.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vpss.h"
#include "rtsp_demo.h"

#define VDEC_CHN   0
#define VENC_CHN   0

static const char *g_file = NULL;
static RK_CODEC_ID_E g_codec = RK_VIDEO_ID_AVC;
static int g_w = 1920, g_h = 1080;
static int g_port = 8554;
static MB_POOL g_vdec_pool = MB_INVALID_POOLID;

static rtsp_demo_handle g_rtsp = NULL;
static rtsp_session_handle g_sess = NULL;
static volatile int g_quit = 0;

static void sigint_handler(int s) { (void)s; g_quit = 1; }

/* ---------- VDEC 帧缓冲池(MB_SOURCE_USER), 复刻官方 SAMPLE_COMM_VDEC_CreateChn ---------- */
static int vdec_init(void) {
    VDEC_MOD_PARAM_S mod;
    VDEC_PIC_BUF_ATTR_S pic;
    MB_PIC_CAL_S cal;
    MB_POOL_CONFIG_S cfg;
    VDEC_CHN_ATTR_S attr;
    VDEC_CHN_PARAM_S param;
    int ret;

    memset(&mod, 0, sizeof(mod));
    mod.enVdecMBSource = MB_SOURCE_USER;
    ret = RK_MPI_VDEC_SetModParam(&mod);
    if (ret) { printf("SetModParam fail %x\n", ret); return -1; }

    memset(&pic, 0, sizeof(pic));
    pic.enCodecType = g_codec;
    pic.stPicBufAttr.u32Width = g_w;
    pic.stPicBufAttr.u32Height = g_h;
    pic.stPicBufAttr.enPixelFormat = RK_FMT_YUV420SP;
#ifdef RV1126B
    pic.stPicBufAttr.enCompMode = COMPRESS_MODE_NONE;
#endif
    memset(&cal, 0, sizeof(cal));
    ret = RK_MPI_CAL_VDEC_GetPicBufferSize(&pic, &cal);
    if (ret) { printf("GetPicBufferSize fail %x\n", ret); return -1; }

    memset(&cfg, 0, sizeof(cfg));
    cfg.u64MBSize = cal.u32MBSize;
    cfg.u32MBCnt = 10;
    cfg.enRemapMode = MB_REMAP_MODE_CACHED;
    cfg.bPreAlloc = RK_TRUE;
    g_vdec_pool = RK_MPI_MB_CreatePool(&cfg);
    if (g_vdec_pool == MB_INVALID_POOLID) { printf("CreatePool fail\n"); return -1; }

    memset(&attr, 0, sizeof(attr));
    attr.enType = g_codec;
    attr.enMode = VIDEO_MODE_STREAM;      /* 官方默认 -m 0 */
    attr.u32PicWidth = g_w;
    attr.u32PicHeight = g_h;
    attr.u32FrameBufCnt = 6;
    attr.u32StreamBufCnt = 8;
    attr.u32FrameBufDepth = 6;
    ret = RK_MPI_VDEC_CreateChn(VDEC_CHN, &attr);
    if (ret) { printf("VDEC create fail %x\n", ret); return -1; }

    memset(&param, 0, sizeof(param));
    param.enType = g_codec;
    ret = RK_MPI_VDEC_SetChnParam(VDEC_CHN, &param);
    if (ret) { printf("SetChnParam fail %x\n", ret); return -1; }

    ret = RK_MPI_VDEC_AttachMbPool(VDEC_CHN, g_vdec_pool);
    if (ret) { printf("AttachMbPool fail %x\n", ret); return -1; }

    ret = RK_MPI_VDEC_StartRecvStream(VDEC_CHN);
    if (ret) { printf("StartRecvStream fail %x\n", ret); return -1; }
    printf("VDEC init OK (%s %dx%d)\n",
           g_codec == RK_VIDEO_ID_AVC ? "h264" : "h265", g_w, g_h);
    return 0;
}

/* ---------- VENC -> RTSP ---------- */
static int venc_init(void) {
    VENC_CHN_ATTR_S attr;
    VENC_RECV_PIC_PARAM_S recv;
    memset(&attr, 0, sizeof(attr));
    attr.stVencAttr.enType = g_codec;
    attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
    attr.stVencAttr.u32PicWidth = g_w;
    attr.stVencAttr.u32PicHeight = g_h;
    attr.stVencAttr.u32VirWidth = g_w;
    attr.stVencAttr.u32VirHeight = g_h;
    attr.stVencAttr.u32StreamBufCnt = 2;
    attr.stVencAttr.u32BufSize = g_w * g_h * 3 / 2;
    attr.stVencAttr.enMirror = MIRROR_NONE;
    if (g_codec == RK_VIDEO_ID_AVC) {
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        attr.stRcAttr.stH264Cbr.u32BitRate = 4000000;
        attr.stRcAttr.stH264Cbr.u32Gop = 60;
        attr.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
    } else {
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        attr.stRcAttr.stH265Cbr.u32BitRate = 4000000;
        attr.stRcAttr.stH265Cbr.u32Gop = 60;
    }
    int ret = RK_MPI_VENC_CreateChn(VENC_CHN, &attr);
    if (ret) { printf("VENC create fail %x\n", ret); return -1; }
    memset(&recv, 0, sizeof(recv));
    recv.s32RecvPicNum = -1;
    RK_MPI_VENC_StartRecvFrame(VENC_CHN, &recv);

    g_rtsp = create_rtsp_demo(g_port);
    if (!g_rtsp) { printf("create_rtsp_demo fail\n"); return -1; }
    g_sess = rtsp_new_session(g_rtsp, "/live/0");
    rtsp_set_video(g_sess, g_codec == RK_VIDEO_ID_AVC ? RTSP_CODEC_ID_VIDEO_H264
                                                       : RTSP_CODEC_ID_VIDEO_H265,
                   NULL, 0);
    printf("VENC init OK, rtsp://<ip>:%d/live/0\n", g_port);
    return 0;
}

/* ---------- 码流线程: 复刻官方 SAMPLE_COMM_VDEC_SendStream(1024B + SYS_CreateMB + 立即 Release) ---------- */
static RK_S32 vdec_free_buf(void *opaque) { if (opaque) free(opaque); return 0; }

static void *feed_thread(void *arg) {
    (void)arg;
    FILE *fp = fopen(g_file, "rb");
    if (!fp) { printf("open %s fail\n", g_file); g_quit = 1; return NULL; }
    int n = 0;
    while (!g_quit) {
        RK_U8 *data = (RK_U8 *)malloc(1024);
        size_t sz = fread(data, 1, 1024, fp);
        if (sz <= 0) { free(data); fseek(fp, 0, SEEK_SET); usleep(2000); continue; }

        MB_EXT_CONFIG_S ext;
        MB_BLK mb = RK_NULL;
        memset(&ext, 0, sizeof(ext));
        ext.pFreeCB = vdec_free_buf;
        ext.pOpaque = data;
        ext.pu8VirAddr = data;
        ext.u64Size = sz;
        if (RK_MPI_SYS_CreateMB(&mb, &ext) != RK_SUCCESS) { free(data); continue; }

        VDEC_STREAM_S st;
        memset(&st, 0, sizeof(st));
        st.u64PTS = 0;
        st.pMbBlk = mb;
        st.u32Len = (RK_U32)sz;
        st.bEndOfStream = RK_FALSE;
        st.bEndOfFrame = RK_FALSE;
        st.bBypassMbBlk = RK_TRUE;
        RK_S32 ret = RK_MPI_VDEC_SendStream(VDEC_CHN, &st, 20);
        n++;
        if (ret < 0 && (n % 200 == 1)) printf("feed SendStream ret=0x%x\n", ret);
        RK_MPI_MB_ReleaseMB(mb);          /* 官方立即释放 */

        /* feed 统计: 确认码流线程是否持续在送(每 2s) */
        static int fc2 = 0; static long fs2 = 0; static struct timespec ft0;
        fc2++; fs2 += sz;
        if (fc2 == 1) clock_gettime(CLOCK_MONOTONIC, &ft0);
        if (fc2 % 50 == 0) {
            struct timespec ft1; clock_gettime(CLOCK_MONOTONIC, &ft1);
            double d = (ft1.tv_sec - ft0.tv_sec) + (ft1.tv_nsec - ft0.tv_nsec) / 1e9;
            printf("[FEED] %d chunks / %.2f MB in %.1fs = %.0f B/s (n=%d)\n",
                   fc2, fs2 / 1e6, d, fs2 / (d > 0 ? d : 1), n);
            fc2 = 0; fs2 = 0;
        }
        usleep(40000);                     /* ~25fps 喂码流节奏 */
    }
    fclose(fp);
    return NULL;
}

/* ---------- 手动分发: VDEC GetFrame -> VENC_SendFrame 直连(绕过 VPSS, 隔离绑定问题) ---------- */
static void *dispatch_thread(void *arg) {
    (void)arg;
    int n = 0, fail = 0, gfail = 0;
    while (!g_quit) {
        VIDEO_FRAME_INFO_S frame;
        memset(&frame, 0, sizeof(frame));
        RK_S32 ret = RK_MPI_VDEC_GetFrame(VDEC_CHN, &frame, 50);
        if (ret != RK_SUCCESS) {
            gfail++;
            if (gfail % 50 == 1)
                printf("[DISP] VDEC_GetFrame fail x%d (total got=%d)\n", gfail, n);
            continue;
        }

        RK_S32 sr = RK_MPI_VENC_SendFrame(VENC_CHN, &frame, 100);
        n++;
        if (n <= 20) {
            // MB_REMAP_MODE_CACHED 池: VDEC 用 DMA 写帧, CPU 读前先失效 cache 才能看真实数据
            RK_MPI_SYS_MmzFlushCache(frame.stVFrame.pMbBlk, RK_FALSE);
            RK_U8 *pv = (RK_U8 *)RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
            if (pv) {
                unsigned sum = 0; int nz = 0;
                int stride = frame.stVFrame.u32VirWidth;
                for (int i = 0; i < 4096; i++) { if (pv[i]) nz++; sum += pv[i]; }
                /* 中间一行(约1/2高)的一个像素, 排除只在头部读 */
                int mid = (int)(frame.stVFrame.u32Height / 2) * stride + (int)(frame.stVFrame.u32Width / 2);
                printf("[FRAME %d] venc_sr=0x%x nz=%d/4096 sum=%u mid=%02x\n",
                       n, sr, nz, sum, pv[mid]);
            }
        }
        if (sr != RK_SUCCESS && (++fail % 50 == 1))
            printf("VENC_SendFrame fail ret=0x%x n=%d\n", sr, n);
        RK_MPI_VDEC_ReleaseFrame(VDEC_CHN, &frame);

        /* 帧率统计: 判断 VDEC 是否持续出帧(非只出前几帧) */
        static int fc = 0; static struct timespec t0;
        fc++;
        if (fc == 1) clock_gettime(CLOCK_MONOTONIC, &t0);
        if (fc % 50 == 0) {
            struct timespec t1; clock_gettime(CLOCK_MONOTONIC, &t1);
            double dt = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
            printf("[FPS] %d frames in %.1fs = %.1f fps (total=%d getfail=%d)\n",
                   fc, dt, fc / dt, n, gfail);
            fc = 0;
        }
    }
    return NULL;
}

/* ---------- VENC 码流 -> RTSP ---------- */
static void *stream_thread(void *arg) {
    (void)arg;
    VENC_STREAM_S st;
    memset(&st, 0, sizeof(st));
    st.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
    int ok = 0, empty = 0;
    while (!g_quit) {
        if (RK_MPI_VENC_GetStream(VENC_CHN, &st, 200000) == RK_SUCCESS) {
            const unsigned char *pd =
                (const unsigned char *)RK_MPI_MB_Handle2VirAddr(st.pstPack->pMbBlk);
            if (g_sess && pd && st.pstPack->u32Len)
                rtsp_tx_video(g_sess, pd, st.pstPack->u32Len, st.pstPack->u64PTS);
            if (g_rtsp) rtsp_do_event(g_rtsp);
            RK_MPI_VENC_ReleaseStream(VENC_CHN, &st);
            ok++;
            /* 统计 VENC 是否持续出包(判断 VPSS->VENC 绑定是否生效) */
            static int vc = 0; static struct timespec vt0;
            vc++;
            if (vc == 1) clock_gettime(CLOCK_MONOTONIC, &vt0);
            if (vc % 50 == 0) {
                struct timespec vt1; clock_gettime(CLOCK_MONOTONIC, &vt1);
                double d = (vt1.tv_sec - vt0.tv_sec) + (vt1.tv_nsec - vt0.tv_nsec) / 1e9;
                printf("[VENC] %d packets in %.1fs = %.1f pps (total ok=%d empty=%d)\n",
                       vc, d, vc / d, ok, empty);
                vc = 0;
            }
        } else {
            empty++;
            /* 每 20 次超时打印一次, 确认 VENC 是否一直拿不到编码包 */
            if (empty % 20 == 1)
                printf("[VENC-TMO] GetStream timeout x%d (got=%d)\n", empty, ok);
        }
    }
    free(st.pstPack);
    return NULL;
}

static void print_usage(const char *p) {
    printf("Usage: %s -f <file> [-e h264|h265] [-w w] [-h h] [-p port]\n", p);
    printf("  -e codec  : h264 or h265 (default h264)\n");
}

int main(int argc, char *argv[]) {
    int opt;
    while ((opt = getopt(argc, argv, "f:e:w:h:p:h")) != -1) {
        switch (opt) {
            case 'f': g_file = optarg; break;
            case 'e': g_codec = !strcmp(optarg, "h265") ? RK_VIDEO_ID_HEVC : RK_VIDEO_ID_AVC; break;
            case 'w': g_w = atoi(optarg); break;
            case 'h': g_h = atoi(optarg); break;
            case 'p': g_port = atoi(optarg); break;
            default: print_usage(argv[0]); return -1;
        }
    }
    if (!g_file) { print_usage(argv[0]); return -1; }

    signal(SIGINT, sigint_handler);

    if (RK_MPI_SYS_Init() != RK_SUCCESS) { printf("SYS_Init fail\n"); return -1; }
    if (vdec_init()) return -1;
    if (venc_init()) return -1;

    pthread_t t_feed, t_disp, t_stream;
    pthread_create(&t_feed, NULL, feed_thread, NULL);
    pthread_create(&t_disp, NULL, dispatch_thread, NULL);
    pthread_create(&t_stream, NULL, stream_thread, NULL);

    while (!g_quit) usleep(100000);

    g_quit = 1;
    pthread_join(t_feed, NULL);
    pthread_join(t_disp, NULL);
    pthread_join(t_stream, NULL);

    RK_MPI_VDEC_StopRecvStream(VDEC_CHN);
    RK_MPI_VDEC_DetachMbPool(VDEC_CHN);
    RK_MPI_VDEC_DestroyChn(VDEC_CHN);
    if (g_vdec_pool != MB_INVALID_POOLID) RK_MPI_MB_DestroyPool(g_vdec_pool);
    RK_MPI_VENC_StopRecvFrame(VENC_CHN);
    RK_MPI_VENC_DestroyChn(VENC_CHN);
    if (g_rtsp) rtsp_del_demo(g_rtsp);
    RK_MPI_SYS_Exit();
    printf("cleanup done\n");
    return 0;
}
