/*
 * Copyright 2025 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <getopt.h>

#include "sample_comm.h"

#define DEFAULT_VDEC_CHN 0
#define DEFAULT_VO_DEV 1
#define DEFAULT_VO_LAYER 5
#define DEFAULT_VO_CHN 0
#define DEFAULT_VPSS_GRP 0
#define DEFAULT_VPSS_CHN 0
#define DEFAULT_H264_FILE_PATH "/data/h264_1280x720"
#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 720
#define BUFFER_SIZE (1024 * 1024)
#define MAX_TIME_OUT_MS   20

#ifndef VIDEO_PROC_DEV_VPSS
#define VIDEO_PROC_DEV_VPSS VIDEO_PROC_DEV_RGA
#endif

typedef struct _rkModeTest {
    RK_BOOL bIfMainThreadQuit;
    RK_BOOL bIfVoThreadQuit;;
    RK_BOOL bIfSendDataThreadQuit;
    RK_BOOL bModuleTestThreadQuit;
    RK_BOOL bModuleTestIfopen;
    RK_S32 s32ModuleTestType;
    RK_S32 s32ModuleTestLoop;
    RK_U32 u32TestFrameCount;
    pthread_t sendFrameThread
} MODE_TEST_S;

typedef struct _rkTestParams {
    RK_S32 s32VdecWidth;
    RK_S32 s32VdecHeight;
    RK_S32 s32VoDisplayWidth;
    RK_S32 s32VoDisplayHeight;
    RK_S32 s32VpssWidth;
    RK_S32 s32VpssHeight;
    VDEC_CHN s32VdecChn;
    RK_CODEC_ID_E enType;
    VIDEO_MODE_E  enMode;
    const char* pCodecFilePath;
    RK_S32 s32TestType;
    RK_S32 s32TestLoop;
    RK_U32 u32TestFrameCount;
} ARGV_PARAMS_S;

typedef struct _rkMpiCtx {
    SAMPLE_VO_CTX_S vo;
    SAMPLE_VDEC_CTX_S vdec;
    SAMPLE_VPSS_CTX_S vpss;
} SAMPLE_MPI_CTX_S;

/* global param */
MODE_TEST_S *gModeTest = {0};
SAMPLE_MPI_CTX_S *g_ctx = {0};
RK_S32 g_exit_result = RK_SUCCESS;
sem_t g_sem_module_test = {0};
pthread_mutex_t g_frame_count_mutex = {0};

static void sigint_handler(int signo) {
    if (signo == SIGINT) {
        g_exit_result = RK_FAILURE;
        if (gModeTest)
            gModeTest->bIfMainThreadQuit = RK_TRUE;
    } else {
        fprintf(stderr, "signal %d\n", signo);
    }
}

// 检查函数返回值，若失败则打印错误信息并退出程序
static void check_ret(RK_S32 ret, const char* func_name) {
    if (ret != RK_SUCCESS) {
        fprintf(stderr, "[%s] failed with error code: 0x%x\n", func_name, ret);
        RK_MPI_SYS_Exit();
        exit(EXIT_FAILURE);
    }
}

// 定义长选项
static RK_CHAR optstr[] = "i:o:f:e:t:l:c:m:v:?:p:";
static struct option long_options[] = {
    {"vdec_input", required_argument, 0, 'i'},
    {"vpss_output", required_argument, 0, 'p'},
    {"vo_output", required_argument, 0, 'o'},
    {"enc_file_path", required_argument, 0, 'f'},
    {"enc_type", required_argument, 0, 'e'},
    {"vdec_mode", required_argument, 0, 'm'},
    {"vdec_chn", required_argument, 0, 'v'},
    {"test_type", required_argument, 0, 't'},
    {"test_loop", required_argument, 0, 'l'},
    {"test_frame_count", required_argument, 0, 'c'},
    {"help", no_argument, 0, '?'},
    {0, 0, 0, 0}
};

struct { int type; char name[128]; } test_type[] = {
    {1, "vpss_resolution"},
    {2, "vpss_switch_dev_type"},
    {3, "ubind_bind"},
    {4, "deinit"},
};

void* send_data(void* arg) {
    ARGV_PARAMS_S *pstTestParams = (ARGV_PARAMS_S *)arg;
    int nal_start = 0;
    FILE* file = fopen(pstTestParams->pCodecFilePath, "rb");
    RK_U8* buffer = (RK_U8*)malloc(BUFFER_SIZE);
    if (file == NULL) {
        perror("Failed to open H264 file");
        return NULL;
    }

    while (!gModeTest->bIfSendDataThreadQuit) {
        if (pstTestParams->enMode == VIDEO_MODE_STREAM) {
            SAMPLE_COMM_VDEC_SendStream(&g_ctx->vdec, file);
        } else if (pstTestParams->enMode == VIDEO_MODE_FRAME) {
            SAMPLE_COMM_VDEC_SendFrame(&g_ctx->vdec, file, buffer, &nal_start);
        } else {
            RK_LOGE("test no support vdec mode:%d now", pstTestParams->enMode);
        }
    }
    if (file)
        fclose(file);
    if (buffer)
        free(buffer);
    return NULL;
}

void *vo_status_thread_func(void *args) {
    SAMPLE_MPI_CTX_S *ctx = (SAMPLE_MPI_CTX_S *)args;
    RK_S32 ret = 0;
    RK_S32 rev_cnt = 0, last_rev_cnt = 0;
    VIDEO_FRAME_INFO_S stFrame;

    while (!gModeTest->bIfVoThreadQuit) {
        rev_cnt = SAMPLE_COMM_VO_GetRevCountByDumpsys(&ctx->vo);

        if (gModeTest->bModuleTestIfopen) {
            if (!last_rev_cnt)
            last_rev_cnt = rev_cnt;
            if (rev_cnt - last_rev_cnt >= gModeTest->u32TestFrameCount) {
                gModeTest->bModuleTestIfopen = 0;
                last_rev_cnt = 0;
                sem_post(&g_sem_module_test);
            }
        }
        RK_LOGI("vo rev cnt: %d \n", rev_cnt);
        // RK_MPI_VO_ReleaseChnFrame(ctx->vo.s32LayerId, ctx->vo.s32ChnId, &stFrame);
        sleep(1);
    }
    return NULL;
}

static void wait_module_test_switch_success(void) {
    gModeTest->bModuleTestIfopen = 1;
	sem_wait(&g_sem_module_test);
}

RK_S32 vpss_resolution() {
    VPSS_CHN_ATTR_S stChnAttr = {0};
    RK_S32 index = 0, ret = 0;
    srand(time(NULL));
    index = rand() % 6;

    ret = RK_MPI_VPSS_GetChnAttr(g_ctx->vpss.s32GrpId, g_ctx->vpss.s32ChnId, &stChnAttr);
    switch (index) {
        case 0:
            stChnAttr.u32Width = 1280;
            stChnAttr.u32Height = 720;
            break;
        case 1:
            stChnAttr.u32Width = 1200;
            stChnAttr.u32Height = 640;
            break;
        case 2:
            stChnAttr.u32Width = 640;
            stChnAttr.u32Height = 640;
            break;
        default:
            break;
    }

    RK_LOGI("vpss resolution switch to: w:%d h:%d", stChnAttr.u32Width, stChnAttr.u32Height);
    return RK_MPI_VPSS_SetChnAttr(g_ctx->vpss.s32GrpId, g_ctx->vpss.s32ChnId, &stChnAttr);
}

RK_S32 vpss_dev_type() {
    RK_S32 ret = 0;
    MPP_CHN_S vdec_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = DEFAULT_VDEC_CHN};
    MPP_CHN_S vpss_chn = {.enModId = RK_ID_VPSS, .s32DevId = DEFAULT_VPSS_GRP, .s32ChnId = DEFAULT_VPSS_CHN};
    MPP_CHN_S vo_chn = {.enModId = RK_ID_VO, .s32DevId = DEFAULT_VO_LAYER, .s32ChnId = DEFAULT_VO_CHN};

    ret = RK_MPI_SYS_UnBind(&vpss_chn, &vo_chn);
    ret |= RK_MPI_SYS_UnBind(&vdec_chn, &vpss_chn);
    if (ret) {
        RK_LOGE("mod ubind fail");
        return -1;
    }

    ret = SAMPLE_COMM_VPSS_DestroyChn(&g_ctx->vpss);
    g_ctx->vpss.enVProcDevType = g_ctx->vpss.enVProcDevType ==
        VIDEO_PROC_DEV_RGA ? VIDEO_PROC_DEV_VPSS : VIDEO_PROC_DEV_RGA;
    ret |= SAMPLE_COMM_VPSS_CreateChn(&g_ctx->vpss);
    if (ret) {
        RK_LOGE("vpss restart fail");
        return -1;
    }

    ret = RK_MPI_SYS_Bind(&vdec_chn, &vpss_chn);
    ret |= RK_MPI_SYS_Bind(&vpss_chn, &vo_chn);
    if (ret) {
        RK_LOGE("mod bind fial");
        return -1;
    }

    return 0;
}

RK_S32 media_bind_ubind() {
    MPP_CHN_S vdec_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = DEFAULT_VDEC_CHN};
    MPP_CHN_S vpss_chn = {.enModId = RK_ID_VPSS, .s32DevId = DEFAULT_VPSS_GRP, .s32ChnId = DEFAULT_VPSS_CHN};
    MPP_CHN_S vo_chn = {.enModId = RK_ID_VO, .s32DevId = DEFAULT_VO_LAYER, .s32ChnId = DEFAULT_VO_CHN};
    RK_MPI_SYS_UnBind(&vpss_chn, &vo_chn);
    RK_MPI_SYS_UnBind(&vdec_chn, &vpss_chn);

    SAMPLE_COMM_VPSS_DestroyChn(&g_ctx->vpss);
    g_ctx->vpss.enVProcDevType = g_ctx->vpss.enVProcDevType ==
        VIDEO_PROC_DEV_RGA ? VIDEO_PROC_DEV_VPSS : VIDEO_PROC_DEV_RGA;
    SAMPLE_COMM_VPSS_CreateChn(&g_ctx->vpss);

    RK_MPI_SYS_Bind(&vdec_chn, &vpss_chn);
    RK_MPI_SYS_Bind(&vpss_chn, &vo_chn);
    return 0;
}

RK_S32 media_init_deinit(ARGV_PARAMS_S *pstTestParams) {
    gModeTest->bIfSendDataThreadQuit = 1;
    pthread_join(gModeTest->sendFrameThread, NULL);

    MPP_CHN_S vdec_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = DEFAULT_VDEC_CHN};
    MPP_CHN_S vpss_chn = {.enModId = RK_ID_VPSS, .s32DevId = DEFAULT_VPSS_GRP, .s32ChnId = DEFAULT_VPSS_CHN};
    MPP_CHN_S vo_chn = {.enModId = RK_ID_VO, .s32DevId = DEFAULT_VO_LAYER, .s32ChnId = DEFAULT_VO_CHN};
    RK_MPI_SYS_UnBind(&vpss_chn, &vo_chn);
    RK_MPI_SYS_UnBind(&vdec_chn, &vpss_chn);

    /* deinit */
    SAMPLE_COMM_VO_DestroyChn(&g_ctx->vo);
    SAMPLE_COMM_VPSS_DestroyChn(&g_ctx->vpss);
    SAMPLE_COMM_VDEC_DestroyChn(&g_ctx->vdec);

    /* init */
    SAMPLE_COMM_VDEC_CreateChn(&g_ctx->vdec);
    SAMPLE_COMM_VPSS_CreateChn(&g_ctx->vpss);
    SAMPLE_COMM_VO_CreateChn(&g_ctx->vo);

    RK_MPI_SYS_Bind(&vdec_chn, &vpss_chn);
    RK_MPI_SYS_Bind(&vpss_chn, &vo_chn);

    gModeTest->bIfSendDataThreadQuit = 0;
    pthread_create(&gModeTest->sendFrameThread, NULL, send_data, pstTestParams);
    return 0;
}

void *module_test_thread_func(void *args) {
    RK_S32 ret = 0;
    RK_S32 testCount = 0;
    ARGV_PARAMS_S *pstTestParams = (ARGV_PARAMS_S *)args;
    gModeTest->s32ModuleTestType = pstTestParams->s32TestType;
    gModeTest->s32ModuleTestLoop = pstTestParams->s32TestLoop;
    gModeTest->u32TestFrameCount = pstTestParams->u32TestFrameCount;

    //pthread 设置线程name
    pthread_setname_np(pthread_self(), "module_test_thread_func");
    RK_LOGI("into module_test_thread_func rk_debug");
    while (!gModeTest->bModuleTestThreadQuit) {

        switch (gModeTest->s32ModuleTestType) {
            case 1:
                ret = vpss_resolution();
                break;
            case 2:
                ret = vpss_dev_type();
                break;
            case 3:
                ret = media_bind_ubind();
                break;
            case 4:
                ret = media_init_deinit(pstTestParams);
                break;
            default:
                break;
        }

        if (ret)
            return NULL;

        wait_module_test_switch_success();
        testCount++;
        RK_LOGI("----------------------------------------");
        RK_LOGI("\t\t\tmode test type:%s total:%d now:%d", test_type[gModeTest->s32ModuleTestType - 1].name,
                gModeTest->s32ModuleTestLoop, testCount);
        if (gModeTest->s32ModuleTestLoop > 0 && testCount >= gModeTest->s32ModuleTestLoop) {
            RK_LOGI("\t\t\tmode test finish, exit");
            break;
        }
        RK_LOGI("----------------------------------------");
    }
    return NULL;
}

/* 接口: 初始化全局变量 */
RK_S32 global_param_init() {
    RK_S32 ret = RK_SUCCESS;
    // 初始化全局变量
    gModeTest = (MODE_TEST_S *)malloc(sizeof(MODE_TEST_S));
    if (gModeTest == NULL) {
        fprintf(stderr, "Failed to allocate memory for gModeTest\n");
        return RK_FAILURE;
    }
    memset(gModeTest, 0, sizeof(MODE_TEST_S));
    g_ctx = (SAMPLE_MPI_CTX_S *)malloc(sizeof(SAMPLE_MPI_CTX_S));
    if (g_ctx == NULL) {
        fprintf(stderr, "Failed to allocate memory for g_ctx\n");
        return RK_FAILURE;
    }
    memset(g_ctx, 0, sizeof(SAMPLE_MPI_CTX_S));
    // 初始化信号量
    sem_init(&g_sem_module_test, 0, 0);
    // 初始化互斥锁
    pthread_mutex_init(&g_frame_count_mutex, NULL);
    return ret;
}

/* 接口: 释放全局变量 */
RK_S32 global_param_deinit() {
    RK_S32 ret = RK_SUCCESS;
    // 释放信号量
    sem_destroy(&g_sem_module_test);
    // 释放互斥锁
    pthread_mutex_destroy(&g_frame_count_mutex);
    // 释放全局变量
    if (gModeTest) {
        free(gModeTest);
        gModeTest = NULL;
    }
    if (g_ctx) {
        free(g_ctx);
        g_ctx = NULL;
    }

    return ret;
}

static void print_usage(const RK_CHAR *name) {
    printf("\nUsage Example:\n");
    printf("\t%s -i 1280x720 -p 1280x720 -o 1080x1920 -f /data/h264_1280x720 -e h264\n\n", name);
    printf("Options:\n");
    printf("\t-i | --vdec_input     : VDEC input resolution, format: widthxheight\n");
    printf("\t-p | --vpss_output    : VPSS output resolution, format: widthxheight\n");
    printf("\t-o | --vo_output      : VO display resolution, format: widthxheight\n");
    printf("\t-f | --enc_file_path  : Input file path\n");
    printf("\t-e | --enc_type       : Encoder type, support: h264/h265/mjpeg\n");
    printf("\t-v | --vdec_chn       : VDEC channel number, default: %d\n", DEFAULT_VDEC_CHN);
    printf("\t-m | --vdec_mode      : VDEC mode, 0:stream mode, 1:frame mode. default: 0\n");
    printf("\t-t | --test_type      : 1.vpss_resolution, 2.vpss_dev_type, 3.ubind_bind, 4:deinit_init\n");
    printf("\t-l | --test_loop      : Test loop count, default: -1(infinite)\n");
    printf("\t-c | --test_frame_count : Test frame count, default: 10\n");
}

static RK_S32 sample_parse_args(int argc, char* argv[], ARGV_PARAMS_S *pstTestParams) {
    if (argc < 2) {
        print_usage(argv[0]);
        return RK_FAILURE;
    }
    
    // 设置默认值
    pstTestParams->s32VdecWidth = DEFAULT_WIDTH;
    pstTestParams->s32VdecHeight = DEFAULT_HEIGHT;
    pstTestParams->s32VpssWidth = DEFAULT_WIDTH;
    pstTestParams->s32VpssHeight = DEFAULT_HEIGHT;
    pstTestParams->s32VoDisplayWidth = DEFAULT_WIDTH;
    pstTestParams->s32VoDisplayHeight = DEFAULT_HEIGHT;
    pstTestParams->s32VdecChn = DEFAULT_VDEC_CHN;
    pstTestParams->enType = RK_VIDEO_ID_AVC;
    pstTestParams->enMode = VIDEO_MODE_STREAM;
    pstTestParams->pCodecFilePath = DEFAULT_H264_FILE_PATH;
    pstTestParams->s32TestType = 0;
    pstTestParams->s32TestLoop = -1;
    pstTestParams->u32TestFrameCount = 10;

    int opt;
    while ((opt = getopt_long(argc, argv, optstr, long_options, NULL)) != -1) {
        switch (opt) {
            case 'i': {
                if (sscanf(optarg, "%dx%d", &pstTestParams->s32VdecWidth, 
                          &pstTestParams->s32VdecHeight) != 2) {
                    RK_LOGE("Invalid vdec resolution format: %s\n", optarg);
                    return RK_FAILURE;
                }
                break;
            }
            case 'p': {
                if (sscanf(optarg, "%dx%d", &pstTestParams->s32VpssWidth, 
                          &pstTestParams->s32VpssHeight) != 2) {
                    RK_LOGE("Invalid vo resolution format: %s\n", optarg);
                    return RK_FAILURE;
                }
                break;
            }
            case 'o': {
                if (sscanf(optarg, "%dx%d", &pstTestParams->s32VoDisplayWidth, 
                          &pstTestParams->s32VoDisplayHeight) != 2) {
                    RK_LOGE("Invalid vo resolution format: %s\n", optarg);
                    return RK_FAILURE;
                }
                break;
            }
            case 'f':
                pstTestParams->pCodecFilePath = optarg;
                break;
            case 'e': {
                if (!strcmp(optarg, "h264")) {
                    pstTestParams->enType = RK_VIDEO_ID_AVC;
                } else if (!strcmp(optarg, "h265")) {
                    pstTestParams->enType = RK_VIDEO_ID_HEVC;
                } else if (!strcmp(optarg, "mjpeg")) {
                    pstTestParams->enType = RK_VIDEO_ID_MJPEG;
                } else {
                    printf("Invalid codec type: %s\n", optarg);
                }
                break;
            }
            case 'v':
                pstTestParams->s32VdecChn = atoi(optarg);
                break;
            case 'm': {
                pstTestParams->enMode = atoi(optarg);
                break;
            }
            case 't':
                pstTestParams->s32TestType = atoi(optarg);
                if (gModeTest) {
                    gModeTest->s32ModuleTestType = pstTestParams->s32TestType;
                }
                break;
            case 'l':
                pstTestParams->s32TestLoop = atoi(optarg);
                if (gModeTest) {
                    gModeTest->s32ModuleTestLoop = pstTestParams->s32TestLoop;
                }
                break;
            case 'c':
                pstTestParams->u32TestFrameCount = atoi(optarg);
                if (gModeTest) {
                    gModeTest->u32TestFrameCount = pstTestParams->u32TestFrameCount;
                }
                break;
            case '?':
            default:
                print_usage(argv[0]);
                return RK_FAILURE;
        }
    }
    printf("vdec w:%d h:%d vo w:%d h:%d\n", pstTestParams->s32VdecWidth, pstTestParams->s32VdecHeight,
           pstTestParams->s32VoDisplayWidth, pstTestParams->s32VoDisplayHeight);
    printf("Codec file path: %s\n", pstTestParams->pCodecFilePath);
    printf("Codec type: %s\n", pstTestParams->enType == RK_VIDEO_ID_AVC ? "h264" : "h265");
    printf("VDEC channel: %d\n", pstTestParams->s32VdecChn);
    printf("VDEC mode: %d\n", pstTestParams->enMode);
    printf("Test type: %d\n", pstTestParams->s32TestType);
    printf("Test loop: %d\n", pstTestParams->s32TestLoop);
    printf("Test frame count: %d\n", pstTestParams->u32TestFrameCount);
    return RK_SUCCESS;
}

int main(int argc, char* argv[]) {
    RK_S32  ret = RK_SUCCESS;
    ARGV_PARAMS_S stTestParams;

    signal(SIGINT, sigint_handler);

    if (sample_parse_args(argc, argv, &stTestParams))
        return RK_FAILURE;

    global_param_init();

    ret = RK_MPI_SYS_Init();
    check_ret(ret, "RK_MPI_SYS_Init");

    g_ctx->vdec.VdChn = stTestParams.s32VdecChn;
    g_ctx->vdec.stChnAttr.enType = stTestParams.enType;
    g_ctx->vdec.stChnAttr.enMode = stTestParams.enMode;
    g_ctx->vdec.stChnAttr.u32PicWidth = stTestParams.s32VdecWidth;
    g_ctx->vdec.stChnAttr.u32PicHeight = stTestParams.s32VdecHeight;
    g_ctx->vdec.stChnAttr.u32FrameBufCnt = 6;
    g_ctx->vdec.stChnAttr.u32StreamBufCnt = 8;
    g_ctx->vdec.stParam.enType = stTestParams.enType;
    ret = SAMPLE_COMM_VDEC_CreateChn(&g_ctx->vdec);
    if (ret) {
        printf("SAMPLE_COMM_VDEC_CreateChn failed\n");
        return RK_FAILURE;
    }

    /* Init VPSS */
    g_ctx->vpss.s32GrpId = 0;
    g_ctx->vpss.s32ChnId = 0;
    /* RGA_device: VIDEO_PROC_DEV_RGA */
    g_ctx->vpss.enVProcDevType = VIDEO_PROC_DEV_RGA;
    g_ctx->vpss.stGrpVpssAttr.enPixelFormat = RK_FMT_YUV420SP;
    g_ctx->vpss.stGrpVpssAttr.enCompressMode = COMPRESS_MODE_NONE; /* no compress */
    g_ctx->vpss.stVpssChnAttr[0].enChnMode = VPSS_CHN_MODE_USER;
    g_ctx->vpss.stVpssChnAttr[0].enCompressMode = COMPRESS_MODE_NONE;
    g_ctx->vpss.stVpssChnAttr[0].enDynamicRange = DYNAMIC_RANGE_SDR8;
    g_ctx->vpss.stVpssChnAttr[0].enPixelFormat = RK_FMT_YUV420SP;
    g_ctx->vpss.stVpssChnAttr[0].stFrameRate.s32SrcFrameRate = -1;
    g_ctx->vpss.stVpssChnAttr[0].stFrameRate.s32DstFrameRate = -1;
    g_ctx->vpss.stVpssChnAttr[0].u32Width = stTestParams.s32VpssWidth;
    g_ctx->vpss.stVpssChnAttr[0].u32Height = stTestParams.s32VpssHeight;
    g_ctx->vpss.stVpssChnAttr[0].u32FrameBufCnt = 4;
    g_ctx->vpss.stVpssChnAttr[0].u32Depth = 0;
    ret = SAMPLE_COMM_VPSS_CreateChn(&g_ctx->vpss);
    check_ret(ret, "RK_MPI_VO_SetPubAttr");

    g_ctx->vo.u32DispBufLen = 4;
    g_ctx->vo.Volayer_mode = VO_LAYER_MODE_VIDEO;
    g_ctx->vo.s32DevId = DEFAULT_VO_DEV;
    g_ctx->vo.s32LayerId = DEFAULT_VO_LAYER;
    g_ctx->vo.s32ChnId = DEFAULT_VO_CHN;
    //pub
    g_ctx->vo.stVoPubAttr.enIntfType = VO_INTF_MIPI;
    g_ctx->vo.stVoPubAttr.enIntfSync = VO_OUTPUT_DEFAULT;
    //layer
    g_ctx->vo.stLayerAttr.enPixFormat = RK_FMT_YUV420SP;
    g_ctx->vo.stLayerAttr.stDispRect.s32X = 0;
    g_ctx->vo.stLayerAttr.stDispRect.s32Y = 0;
    g_ctx->vo.stLayerAttr.stDispRect.u32Width = stTestParams.s32VoDisplayWidth;
    g_ctx->vo.stLayerAttr.stDispRect.u32Height = stTestParams.s32VoDisplayHeight;
    g_ctx->vo.stLayerAttr.stImageSize.u32Width = stTestParams.s32VoDisplayWidth;
    g_ctx->vo.stLayerAttr.stImageSize.u32Height = stTestParams.s32VoDisplayHeight;
    //chn
    g_ctx->vo.stChnAttr.stRect.s32X = 0;
    g_ctx->vo.stChnAttr.stRect.s32Y = 0;
    g_ctx->vo.stChnAttr.stRect.u32Width = stTestParams.s32VoDisplayWidth;
    g_ctx->vo.stChnAttr.stRect.u32Height = stTestParams.s32VoDisplayHeight;
    ret = SAMPLE_COMM_VO_CreateChn(&g_ctx->vo);
    if (ret) {
        printf("SAMPLE_COMM_VO_CreateChn failed\n");
        return RK_FAILURE;
    }

    MPP_CHN_S vdec_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = DEFAULT_VDEC_CHN};
    MPP_CHN_S vpss_chn = {.enModId = RK_ID_VPSS, .s32DevId = DEFAULT_VPSS_GRP, .s32ChnId = DEFAULT_VPSS_CHN};
    MPP_CHN_S vo_chn = {.enModId = RK_ID_VO, .s32DevId = DEFAULT_VO_LAYER, .s32ChnId = DEFAULT_VO_CHN};
    ret = RK_MPI_SYS_Bind(&vdec_chn, &vpss_chn);
    check_ret(ret, "RK_MPI_SYS_Bind(VDEC to VPSS)");
    ret = RK_MPI_SYS_Bind(&vpss_chn, &vo_chn);
    check_ret(ret, "RK_MPI_SYS_Bind(VPSS to VO)");

    ret = pthread_create(&gModeTest->sendFrameThread, NULL, send_data, &stTestParams);
    if (ret != 0) {
        perror("Failed to create thread");
        RK_MPI_VDEC_StopRecvStream(stTestParams.s32VdecChn);
        RK_MPI_VDEC_DestroyChn(stTestParams.s32VdecChn);
        RK_MPI_SYS_Exit();
        return EXIT_FAILURE;
    }

    pthread_t vo_status_thread;
    ret = pthread_create(&vo_status_thread, NULL, vo_status_thread_func, g_ctx);
    check_ret(ret, "pthread_create vo_status_thread_func");

    pthread_t module_test_thread;
    if (stTestParams.s32TestType) {
        ret = pthread_create(&module_test_thread, NULL, module_test_thread_func, &stTestParams);
        check_ret(ret, "pthread_create module_test_thread_func");
    }

    while (!gModeTest->bIfMainThreadQuit) {
        usleep(10*1000);
    }

    if (stTestParams.s32TestType) {
        gModeTest->bModuleTestThreadQuit = 1;
        sem_post(&g_sem_module_test);
        ret = pthread_join(module_test_thread, NULL);
        if (ret != 0) {
            perror("Failed to join thread");
        }
    }

    gModeTest->bIfVoThreadQuit = 1;
    ret = pthread_join(vo_status_thread, NULL);
    if (ret != 0) {
        perror("Failed to join thread");
    }

    gModeTest->bIfSendDataThreadQuit = 1;
    ret = pthread_join(gModeTest->sendFrameThread, NULL);
    if (ret != 0) {
        perror("Failed to join thread");
    }

    ret = RK_MPI_SYS_UnBind(&vpss_chn, &vo_chn);
    check_ret(ret, "RK_MPI_SYS_UnBind(VPSS to VO)");
     //destroy vo
     SAMPLE_COMM_VO_DestroyChn(&g_ctx->vo);
    //ubind vdec and vpss
    ret = RK_MPI_SYS_UnBind(&vdec_chn, &vpss_chn);
    check_ret(ret, "RK_MPI_SYS_UnBind(VDEC to VPSS)");

    /* Destroy VPSS */
    ret = SAMPLE_COMM_VPSS_DestroyChn(&g_ctx->vpss);
    check_ret(ret, "SAMPLE_COMM_VPSS_DestroyChn");

    /* destroy vdec */
    SAMPLE_COMM_VDEC_DestroyChn(&g_ctx->vdec);

    ret = RK_MPI_SYS_Exit();
    check_ret(ret, "RK_MPI_SYS_Exit");

    ret = global_param_deinit();
    check_ret(ret, "global_param_deinit");

    return 0;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
