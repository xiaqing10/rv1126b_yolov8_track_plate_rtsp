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
#define DEFAULT_H264_FILE_PATH "/data/h264_1280x720"
#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 720
#define BUFFER_SIZE (1024 * 1024)
#define MAX_TIME_OUT_MS   20

typedef struct _rkModeTest {
    RK_BOOL bIfMainThreadQuit;
    RK_BOOL bIfSendDataThreadQuit;
    RK_BOOL bIfVoThreadQuit;
    RK_BOOL bModuleTestThreadQuit;
    RK_BOOL bModuleTestIfopen;
    RK_S32 s32ModuleTestType;
    RK_S32 s32ModuleTestLoop;
    RK_U32 u32TestFrameCount;
    pthread_t sendFrameThread
} MODE_TEST_S;

typedef struct _rkArgvParams {
    RK_S32 s32VdecWidth;
    RK_S32 s32VdecHeight;
    RK_S32 s32VoDisplayWidth;
    RK_S32 s32VoDisplayHeight;
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
} SAMPLE_MPI_CTX_S;

/* global param */
MODE_TEST_S *gModeTest = NULL;
SAMPLE_MPI_CTX_S *g_ctx = NULL;
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

static void check_ret(RK_S32 ret, const char* func_name) {
    if (ret != RK_SUCCESS) {
        fprintf(stderr, "[%s] failed with error code: 0x%x\n", func_name, ret);
        RK_MPI_SYS_Exit();
        exit(EXIT_FAILURE);
    }
}

static RK_CHAR optstr[] = "i:o:f:e:t:l:c:m:v:?:";
static struct option long_options[] = {
    {"vdec_input", required_argument, 0, 'i'},
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
    {1, "vdec_mode_switch"},
    {2, "vdec_attach_pool"},
    {3, "vdec_reset"},
    {4, "vo_display_mode"},
    {5, "ubind_bind"},
    {6, "deinit"},
};

void* send_data(void* arg) {
    ARGV_PARAMS_S *pstTestParams = (ARGV_PARAMS_S *)arg;
    int nal_start = 0;
    RK_U8* buffer = (RK_U8*)malloc(BUFFER_SIZE);
    FILE* file = fopen(pstTestParams->pCodecFilePath, "rb");
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

RK_S32 vdec_mode_switch(ARGV_PARAMS_S *pstTestParams) {
    RK_S32 ret = 0;
    RK_S32 mode = 0;
    srand(time(NULL));
    mode = rand() % 5;

    gModeTest->bIfSendDataThreadQuit = 1;
    pthread_join(gModeTest->sendFrameThread, NULL);

    MPP_CHN_S src_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = g_ctx->vdec.VdChn};
    MPP_CHN_S dest_chn = {.enModId = RK_ID_VO, .s32DevId = g_ctx->vo.s32LayerId, .s32ChnId = g_ctx->vo.s32ChnId};
    RK_MPI_SYS_UnBind(&src_chn, &dest_chn);
    SAMPLE_COMM_VDEC_DestroyChn(&g_ctx->vdec);

    switch (mode) {
        case 0: // stream mode
            g_ctx->vdec.stChnAttr.enMode = VIDEO_MODE_STREAM;
            RK_LOGI("vdec switch to steam mode");
            break;
        case 1: // frame mode
            g_ctx->vdec.stChnAttr.enMode = VIDEO_MODE_FRAME;
            RK_LOGI("vdec switch to frame mode");
            break;
        case 2: //h264
            g_ctx->vdec.stChnAttr.enType = RK_VIDEO_ID_AVC;
            pstTestParams->pCodecFilePath = "/data/h264_1280x720";
            RK_LOGI("vdec switch to h264");
            break;
        case 3: //h265
            g_ctx->vdec.stChnAttr.enType = RK_VIDEO_ID_HEVC;
            pstTestParams->pCodecFilePath = "/data/h265_1280x720";
            RK_LOGI("vdec switch to h265");
            break;
        case 4: //mjpeg
            // g_ctx->vdec.stChnAttr.enType = RK_VIDEO_ID_MJPEG;
            // pstTestParams->pCodecFilePath = "/data/mjpeg_1280x720";
            // RK_LOGI("vdec switch to mjpeg");
            break;
        case 5: //compress mode
            // g_ctx->vdec.stChnAttr.enCompressMode = COMPRESS_AFBC_16x16;
            break;
        default:
            break;
    }

    if (g_ctx->vdec.stChnAttr.enMode == VIDEO_MODE_FRAME) {
        g_ctx->vdec.stChnAttr.enType = RK_VIDEO_ID_AVC;
        pstTestParams->enMode = VIDEO_MODE_FRAME;
        pstTestParams->pCodecFilePath = "/data/h264_1280x720";
        RK_LOGI("frame mode, force to h264 type");
    } else
        pstTestParams->enMode = VIDEO_MODE_STREAM;

    SAMPLE_COMM_VDEC_CreateChn(&g_ctx->vdec);
    RK_MPI_SYS_Bind(&src_chn, &dest_chn);


    gModeTest->bIfSendDataThreadQuit = 0;
    pthread_create(&gModeTest->sendFrameThread, NULL, send_data, pstTestParams);

    return ret;
}

RK_S32 vdec_attach_pool(ARGV_PARAMS_S *pstTestParams) {

    gModeTest->bIfSendDataThreadQuit = 1;
    pthread_join(gModeTest->sendFrameThread, NULL);

    MPP_CHN_S src_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = g_ctx->vdec.VdChn};
    MPP_CHN_S dest_chn = {.enModId = RK_ID_VO, .s32DevId = g_ctx->vo.s32LayerId, .s32ChnId = g_ctx->vo.s32ChnId};
    RK_MPI_SYS_UnBind(&src_chn, &dest_chn);
    SAMPLE_COMM_VDEC_DestroyChn(&g_ctx->vdec);

    g_ctx->vdec.s32Pool = g_ctx->vdec.s32Pool ? 0 : 1;

    SAMPLE_COMM_VDEC_CreateChn(&g_ctx->vdec);
    RK_MPI_SYS_Bind(&src_chn, &dest_chn);
    gModeTest->bIfSendDataThreadQuit = 0;
    pthread_create(&gModeTest->sendFrameThread, NULL, send_data, pstTestParams);
    return 0;
}

RK_S32 vdec_reset(ARGV_PARAMS_S *pstTestParams) {

    gModeTest->bIfSendDataThreadQuit = 1;
    pthread_join(gModeTest->sendFrameThread, NULL);

    MPP_CHN_S src_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = g_ctx->vdec.VdChn};
    MPP_CHN_S dest_chn = {.enModId = RK_ID_VO, .s32DevId = g_ctx->vo.s32LayerId, .s32ChnId = g_ctx->vo.s32ChnId};
    RK_MPI_SYS_UnBind(&src_chn, &dest_chn);

    RK_MPI_VDEC_ResetChn(g_ctx->vdec.VdChn);

    RK_MPI_SYS_Bind(&src_chn, &dest_chn);
    gModeTest->bIfSendDataThreadQuit = 0;
    pthread_create(&gModeTest->sendFrameThread, NULL, send_data, pstTestParams);
    return 0;
}

RK_S32 vo_change_layer_resolution() {
    RK_S32 ret = 0, mode;
    RECT_S stRect;
    srand(time(NULL));
    mode = rand() % 3;

    switch (mode) {
        case 0:
            stRect.s32X = 0;
            stRect.s32Y = 0;
            stRect.u32Width = 720;
            stRect.u32Height = 1280;
            break;
        case 1:
            stRect.s32X = 128;
            stRect.s32Y = 128;
            stRect.u32Width = 640;
            stRect.u32Height = 640;
            break;
        case 2:
            stRect.s32X = 720;
            stRect.s32Y = 1280;
            stRect.u32Width = 128;
            stRect.u32Height = 128;
            break;
        default:
            break;
    }
    RK_LOGI("change to x:%d y:%d  w:%d  h:%d", stRect.s32X,
            stRect.s32Y, stRect.u32Width, stRect.u32Height);

    #ifdef RV1126B
    RK_MPI_VO_SetLayerCleanMode(g_ctx->vo.s32LayerId, VO_LAYER_CLEAN_MODE_ONCE);
    return RK_MPI_VO_SetLayerDispRect(g_ctx->vo.s32LayerId, &stRect);
    #else
    return 0;
    #endif
}

RK_S32 vdec_vo_bind_unbind() {
    MPP_CHN_S src_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = g_ctx->vdec.VdChn};
    MPP_CHN_S dest_chn = {.enModId = RK_ID_VO, .s32DevId = g_ctx->vo.s32LayerId, .s32ChnId = g_ctx->vo.s32ChnId};
    RK_MPI_SYS_UnBind(&src_chn, &dest_chn);
    return RK_MPI_SYS_Bind(&src_chn, &dest_chn);
}

RK_S32 vdec_vo_deinit(ARGV_PARAMS_S *pstTestParams) {
    gModeTest->bIfSendDataThreadQuit = 1;
    pthread_join(gModeTest->sendFrameThread, NULL);

    MPP_CHN_S src_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = g_ctx->vdec.VdChn};
    MPP_CHN_S dest_chn = {.enModId = RK_ID_VO, .s32DevId = g_ctx->vo.s32LayerId, .s32ChnId = g_ctx->vo.s32ChnId};
    RK_MPI_SYS_UnBind(&src_chn, &dest_chn);
    SAMPLE_COMM_VO_DestroyChn(&g_ctx->vo);
    SAMPLE_COMM_VDEC_DestroyChn(&g_ctx->vdec);
    /* init */
    SAMPLE_COMM_VDEC_CreateChn(&g_ctx->vdec);
    SAMPLE_COMM_VO_CreateChn(&g_ctx->vo);
    RK_MPI_SYS_Bind(&src_chn, &dest_chn);

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
                ret = vdec_mode_switch(pstTestParams);
                break;
            case 2:
                ret = vdec_attach_pool(pstTestParams);
                break;
            case 3:
                ret = vdec_reset(pstTestParams);
                break;
            case 4:
                ret = vo_change_layer_resolution();
                break;
            case 5:
                ret = vdec_vo_bind_unbind();
                break;
            case 6:
                ret = vdec_vo_deinit(pstTestParams);
                break;
            default:
                break;
        }

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

RK_S32 global_param_init() {
    RK_S32 ret = RK_SUCCESS;

    gModeTest = (MODE_TEST_S *)malloc(sizeof(MODE_TEST_S));
    if (gModeTest == NULL) {
        fprintf(stderr, "Failed to allocate memory for MODE_TEST_S\n");
        return RK_FAILURE;
    }
    memset(gModeTest, 0, sizeof(MODE_TEST_S));
    g_ctx = (SAMPLE_MPI_CTX_S *)malloc(sizeof(SAMPLE_MPI_CTX_S));
    if (g_ctx == NULL) {
        fprintf(stderr, "Failed to allocate memory for g_ctx\n");
        return RK_FAILURE;
    }
    memset(g_ctx, 0, sizeof(SAMPLE_MPI_CTX_S));

    sem_init(&g_sem_module_test, 0, 0);
    pthread_mutex_init(&g_frame_count_mutex, NULL);
    return ret;
}

RK_S32 global_param_deinit() {
    RK_S32 ret = RK_SUCCESS;

    sem_destroy(&g_sem_module_test);
    pthread_mutex_destroy(&g_frame_count_mutex);

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
    printf("\t%s -i 1280x720 -o 1080x1920 -f /data/h264_1280x720 -e h264\n\n", name);
    printf("Options:\n");
    printf("\t-i | --vdec_input     : VDEC input resolution, format: widthxheight\n");
    printf("\t-o | --vo_output      : VO display resolution, format: widthxheight\n");
    printf("\t-f | --enc_file_path  : Input file path\n");
    printf("\t-e | --enc_type       : Encoder type, support: h264/h265/mjpeg\n");
    printf("\t-v | --vdec_chn       : VDEC channel number, default: %d\n", DEFAULT_VDEC_CHN);
    printf("\t-m | --vdec_mode      : VDEC mode, 0:stream mode, 1:frame mode(demo only h264) default: 0\n");
    printf("\t-t | --test_type      : 1:mode random switch(stream mode, frame mode, h264, h265, mjpeg, compressed data),\n");
    printf("\t                        2: vdec attach pool, 3: vdec reset, 4: vo resolution\n");
    printf("\t                        5: bind-unbind, 6: deinit, default:0\n");
    printf("\t-l | --test_loop      : Test loop count, default: -1(infinite)\n");
    printf("\t-c | --test_frame_count : Test frame count, default: 10\n");
}

static RK_S32 sample_parse_args(int argc, char* argv[], ARGV_PARAMS_S *pstTestParams) {
    if (argc < 2) {
        print_usage(argv[0]);
        return RK_FAILURE;
    }

    pstTestParams->s32VdecWidth = DEFAULT_WIDTH;
    pstTestParams->s32VdecHeight = DEFAULT_HEIGHT;
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
                    RK_LOGE("Invalid resolution format: %s\n", optarg);
                    return RK_FAILURE;
                }
                break;
            }
            case 'o': {
                if (sscanf(optarg, "%dx%d", &pstTestParams->s32VoDisplayWidth, 
                          &pstTestParams->s32VoDisplayHeight) != 2) {
                    RK_LOGE("Invalid display resolution format: %s\n", optarg);
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

    MPP_CHN_S src_chn = {.enModId = RK_ID_VDEC, .s32DevId = 0, .s32ChnId = stTestParams.s32VdecChn};
    MPP_CHN_S dest_chn = {.enModId = RK_ID_VO, .s32DevId = g_ctx->vo.s32LayerId, .s32ChnId = g_ctx->vo.s32ChnId}; // 设备层一般通道号为 0
    ret = RK_MPI_SYS_Bind(&src_chn, &dest_chn);
    check_ret(ret, "RK_MPI_SYS_Bind");

    ret = pthread_create(&gModeTest->sendFrameThread, NULL, send_data, &stTestParams);
    if (ret != 0) {
        perror("Failed to create thread");
        SAMPLE_COMM_VDEC_DestroyChn(&g_ctx->vdec);
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

    ret = RK_MPI_SYS_UnBind(&src_chn, &dest_chn);
    check_ret(ret, "RK_MPI_SYS_UnBind");

    //destroy vo
    SAMPLE_COMM_VO_DestroyChn(&g_ctx->vo);
    //destroy vdec
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
