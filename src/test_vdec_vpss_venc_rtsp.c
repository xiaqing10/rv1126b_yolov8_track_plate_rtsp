/*
 * test_vdec_vpss_venc_rtsp.c
 *
 * 改造官方 example/test/sample_vdec_vpss_vo_stresstest.c:
 *   - 保留官方 VDEC/VPSS 初始化与 RK_MPI_SYS_Bind(VDEC->VPSS) 绑定
 *   - 保留官方紧循环喂流 fread(1024)->SendStream->立即ReleaseMB, 无 sleep
 *     (这是画面能否持续解码的关键, 之前测试 usleep(40ms) 导致 VDEC 缓冲积满几乎不解码)
 *   - 仅把输出端 VO/MIPI 替换为 VENC -> RTSP 推流
 *
 * 用法:
 *   test_vdec_vpss_venc_rtsp -f /userdata/xxx.h264 -e h264 -w 1920 -h 1080 -p 8554
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <getopt.h>
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
#define VPSS_GRP   0
#define VPSS_CHN   0
#define VENC_CHN   0

#define MAX_TIME_OUT_MS 20
#define FEED_READ_SIZE  1024

static const char *g_file = NULL;
static RK_CODEC_ID_E g_codec = RK_VIDEO_ID_AVC;
static int g_w = 1920, g_h = 1080;
static int g_port = 8554;
static MB_POOL g_vdec_pool = MB_INVALID_POOLID;

static volatile int g_quit = 0;
static rtsp_demo_handle g_rtsp = NULL;
static rtsp_session_handle g_sess = NULL;

static void sigint_handler(int s) { (void)s; g_quit = 1; }

/* ---------------- VDEC init (复刻官方 SAMPLE_COMM_VDEC_CreateChn, MB_SOURCE_USER 帧池) ---------------- */
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
    attr.enMode = VIDEO_MODE_STREAM;
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
    printf("VDEC init OK (chn %d)\n", VDEC_CHN);
    return 0;
}

/* ---------------- VPSS init (复刻官方 SAMPLE_COMM_VPSS_CreateChn, RGA, 源分辨率) ---------------- */
static int vpss_init(void) {
    VPSS_GRP_ATTR_S gattr;
    VPSS_CHN_ATTR_S cattr;
    memset(&gattr, 0, sizeof(gattr));
    gattr.u32MaxW = g_w;
    gattr.u32MaxH = g_h;
    gattr.enPixelFormat = RK_FMT_YUV420SP;
    gattr.stFrameRate.s32SrcFrameRate = -1;
    gattr.stFrameRate.s32DstFrameRate = -1;
    gattr.enCompressMode = COMPRESS_MODE_NONE;
#ifdef RV1126B
    gattr.enVProcDev = VIDEO_PROC_DEV_RGA;
    gattr.u32MaxQueue = 10;
#endif
    int ret = RK_MPI_VPSS_CreateGrp(VPSS_GRP, &gattr);
    if (ret) { printf("VPSS CreateGrp fail %x\n", ret); return -1; }

    memset(&cattr, 0, sizeof(cattr));
    cattr.enChnMode = VPSS_CHN_MODE_USER;
    cattr.enCompressMode = COMPRESS_MODE_NONE;
    cattr.enDynamicRange = DYNAMIC_RANGE_SDR8;
    cattr.enPixelFormat = RK_FMT_YUV420SP;
    cattr.stFrameRate.s32SrcFrameRate = 25;
    cattr.stFrameRate.s32DstFrameRate = 25;   /* 限制 VPSS 输出 25fps, 避免 VENC 超速编码 -> 每帧码率不足马赛克 */
    cattr.u32Width = g_w;
    cattr.u32Height = g_h;
    cattr.u32FrameBufCnt = 4;
    cattr.u32Depth = 0;
    ret = RK_MPI_VPSS_SetChnAttr(VPSS_GRP, VPSS_CHN, &cattr);
    if (ret) { printf("VPSS SetChnAttr fail %x\n", ret); return -1; }
    ret = RK_MPI_VPSS_EnableChn(VPSS_GRP, VPSS_CHN);
    if (ret) { printf("VPSS EnableChn fail %x\n", ret); return -1; }
    ret = RK_MPI_VPSS_StartGrp(VPSS_GRP);
    if (ret) { printf("VPSS StartGrp fail %x\n", ret); return -1; }
    printf("VPSS init OK (RGA)\n");
    return 0;
}

/* ---------------- VENC init + RTSP ---------------- */
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
    attr.stVencAttr.u32StreamBufCnt = 4;
    attr.stVencAttr.u32BufSize = g_w * g_h * 3 / 2;
    attr.stVencAttr.enMirror = MIRROR_NONE;
    if (g_codec == RK_VIDEO_ID_AVC) {
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        attr.stRcAttr.stH264Cbr.u32BitRate = 8000000;   /* 1080p 动态视频 8Mbps, 避免块效应 */
        attr.stRcAttr.stH264Cbr.u32Gop = 60;
        attr.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
    } else {
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        attr.stRcAttr.stH265Cbr.u32BitRate = 8000000;
        attr.stRcAttr.stH265Cbr.u32Gop = 60;
    }
    if (RK_MPI_VENC_CreateChn(VENC_CHN, &attr)) { printf("VENC create fail\n"); return -1; }
    recv.s32RecvPicNum = -1;
    RK_MPI_VENC_StartRecvFrame(VENC_CHN, &recv);

    g_rtsp = create_rtsp_demo(g_port);
    if (!g_rtsp) { printf("create_rtsp_demo fail\n"); return -1; }
    g_sess = rtsp_new_session(g_rtsp, "/live/0");
    rtsp_set_video(g_sess, g_codec == RK_VIDEO_ID_AVC ? RTSP_CODEC_ID_VIDEO_H264
                                                       : RTSP_CODEC_ID_VIDEO_H265, NULL, 0);
    printf("VENC init OK, rtsp://<ip>:%d/live/0\n", g_port);
    return 0;
}

/* ---------------- 官方紧循环喂流 (fread 1024 -> SendStream -> 立即 ReleaseMB, 无 sleep) ---------------- */
static RK_S32 vdec_free_buf(void *opaque) { if (opaque) free(opaque); return 0; }

static void *send_data(void *arg) {
    (void)arg;
    FILE *fp = fopen(g_file, "rb");
    if (!fp) { printf("open %s fail\n", g_file); return NULL; }
    long total = 0; int n = 0; struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (!g_quit) {
        RK_U8 *data = (RK_U8 *)malloc(FEED_READ_SIZE);
        if (!data) break;
        int sz = (int)fread(data, 1, FEED_READ_SIZE, fp);
        if (sz <= 0) { fseek(fp, 0, SEEK_SET); free(data); continue; }

        MB_EXT_CONFIG_S ext;
        memset(&ext, 0, sizeof(ext));
        ext.pFreeCB = vdec_free_buf;
        ext.pOpaque = data;
        ext.pu8VirAddr = data;
        ext.u64Size = sz;
        MB_BLK mb = NULL;
        RK_MPI_SYS_CreateMB(&mb, &ext);

        VDEC_STREAM_S st;
        memset(&st, 0, sizeof(st));
        st.u64PTS = 0;
        st.pMbBlk = mb;
        st.u32Len = sz;
        st.bEndOfStream = RK_FALSE;
        st.bEndOfFrame = RK_FALSE;
        st.bBypassMbBlk = RK_TRUE;
        RK_MPI_VDEC_SendStream(VDEC_CHN, &st, MAX_TIME_OUT_MS);
        RK_MPI_MB_ReleaseMB(mb);   /* 官方立即释放, 无 sleep */

        total += sz; n++;
        if (n % 2000 == 0) {
            clock_gettime(CLOCK_MONOTONIC, &t1);
            double d = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
            printf("[FEED] %d chunks / %.1f MB in %.1fs = %.0f B/s\n", n, total / 1e6, d, total / (d > 0 ? d : 1));
        }
    }
    fclose(fp);
    return NULL;
}

/* ---------------- VENC 编码 -> RTSP 推流 ---------------- */
static void *stream_thread(void *arg) {
    (void)arg;
    VENC_STREAM_S st;
    memset(&st, 0, sizeof(st));
    st.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
    int ok = 0, empty = 0;
    uint64_t pts = 0;             /* VENC u64PTS 恒为 0(喂流设 0), 播放器遇 PTS=0 只渲染首帧不刷新, 故自增 PTS */
    while (!g_quit) {
        if (RK_MPI_VENC_GetStream(VENC_CHN, &st, 100000) == RK_SUCCESS) {
            const unsigned char *pd =
                (const unsigned char *)RK_MPI_MB_Handle2VirAddr(st.pstPack->pMbBlk);
            pts += 40000;         /* 25fps: 每帧 40ms = 40000us */
            if (g_sess && pd && st.pstPack->u32Len)
                rtsp_tx_video(g_sess, pd, st.pstPack->u32Len, pts);
            if (g_rtsp) rtsp_do_event(g_rtsp);
            RK_MPI_VENC_ReleaseStream(VENC_CHN, &st);
            ok++;
            /* 检测 VENC 是否在编码不同帧: 打印每包 PTS, 若恒定说明 VENC 重复编码同一帧 */
            if (ok <= 30 || ok % 100 == 1)
                printf("[VENC] pkt %d pts=%llu len=%u\n", ok,
                       (unsigned long long)st.pstPack->u64PTS, st.pstPack->u32Len);
        } else {
            empty++;
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
    if (vpss_init()) return -1;
    if (venc_init()) return -1;

    /* 官方绑定: VDEC->VPSS, VPSS->VENC (官方是 VPSS->VO, 我们只换输出端) */
    MPP_CHN_S vdec_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = VDEC_CHN};
    MPP_CHN_S vpss_chn = {.enModId = RK_ID_VPSS, .s32DevId = VPSS_GRP, .s32ChnId = VPSS_CHN};
    MPP_CHN_S venc_chn = {.enModId = RK_ID_VENC, .s32DevId = 0, .s32ChnId = VENC_CHN};
    if (RK_MPI_SYS_Bind(&vdec_chn, &vpss_chn)) { printf("Bind VDEC->VPSS fail\n"); return -1; }
    if (RK_MPI_SYS_Bind(&vpss_chn, &venc_chn)) { printf("Bind VPSS->VENC fail\n"); return -1; }
    printf("media init OK (VDEC %dx%d -> VPSS %dx%d -> VENC -> RTSP)\n", g_w, g_h, g_w, g_h);

    pthread_t t_feed, t_stream;
    pthread_create(&t_feed, NULL, send_data, NULL);
    pthread_create(&t_stream, NULL, stream_thread, NULL);

    while (!g_quit) usleep(100000);

    g_quit = 1;
    pthread_join(t_feed, NULL);
    pthread_join(t_stream, NULL);

    RK_MPI_VDEC_StopRecvStream(VDEC_CHN);
    RK_MPI_VDEC_DetachMbPool(VDEC_CHN);
    RK_MPI_VDEC_DestroyChn(VDEC_CHN);
    if (g_vdec_pool != MB_INVALID_POOLID) RK_MPI_MB_DestroyPool(g_vdec_pool);
    RK_MPI_VPSS_DisableChn(VPSS_GRP, VPSS_CHN);
    RK_MPI_VPSS_StopGrp(VPSS_GRP);
    RK_MPI_VPSS_DestroyGrp(VPSS_GRP);
    RK_MPI_VENC_StopRecvFrame(VENC_CHN);
    RK_MPI_VENC_DestroyChn(VENC_CHN);
    if (g_rtsp) rtsp_del_demo(g_rtsp);
    RK_MPI_SYS_Exit();
    printf("cleanup done\n");
    return 0;
}
