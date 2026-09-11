/*
 * Copyright 2025 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
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

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/poll.h>
#include <unistd.h>

#include "sample_comm.h"

RK_S32 SAMPLE_COMM_VDEC_CreateChn(SAMPLE_VDEC_CTX_S *ctx) {
    RK_S32 ret = RK_SUCCESS;

    if (ctx->bEnableMbPool) {
        VDEC_MOD_PARAM_S vdec_mod_param;
        VDEC_PIC_BUF_ATTR_S stVdecPicBufAttr;
        MB_PIC_CAL_S stMbPicCalResult;
        MB_POOL_CONFIG_S stMbPoolCfg;
        memset(&vdec_mod_param, 0, sizeof(VDEC_MOD_PARAM_S));
        vdec_mod_param.enVdecMBSource = MB_SOURCE_USER;
        ret = RK_MPI_VDEC_SetModParam(&vdec_mod_param);
        if (ret) {
            RK_LOGE("RK_MPI_VDEC_SetModParam chn:%d fail %x", ctx->VdChn, ret);
            return ret;
        }
        memset(&stVdecPicBufAttr, 0, sizeof(VDEC_PIC_BUF_ATTR_S));
        memset(&stMbPicCalResult, 0, sizeof(MB_PIC_CAL_S));
        stVdecPicBufAttr.enCodecType = ctx->stChnAttr.enType;
        stVdecPicBufAttr.stPicBufAttr.u32Width = ctx->stChnAttr.u32PicWidth;
        stVdecPicBufAttr.stPicBufAttr.u32Height = ctx->stChnAttr.u32PicHeight;
        #ifdef RV1126B
        stVdecPicBufAttr.stPicBufAttr.enPixelFormat = (PIXEL_FORMAT_E)ctx->stChnAttr.enPixelFormat;
        stVdecPicBufAttr.stPicBufAttr.enCompMode = (COMPRESS_MODE_E)ctx->stChnAttr.enCompressMode;
        #endif
        ret = RK_MPI_CAL_VDEC_GetPicBufferSize(&stVdecPicBufAttr, &stMbPicCalResult);
        if (ret != RK_SUCCESS) {
            RK_LOGE("get picture buffer size failed. err 0x%x", ret);
            return ret;
        }

        memset(&stMbPoolCfg, 0, sizeof(MB_POOL_CONFIG_S));

        stMbPoolCfg.u64MBSize = stMbPicCalResult.u32MBSize;
        stMbPoolCfg.u32MBCnt  = 10;
        stMbPoolCfg.enRemapMode = MB_REMAP_MODE_CACHED;
        stMbPoolCfg.bPreAlloc = RK_TRUE;
        ctx->s32Pool = RK_MPI_MB_CreatePool(&stMbPoolCfg);
        if (ctx->s32Pool == MB_INVALID_POOLID) {
            RK_LOGE("create pool failed!");
            return -1;
        }

    }

    ret = RK_MPI_VDEC_CreateChn(ctx->VdChn, &ctx->stChnAttr);
    if (ret) {
        RK_LOGE("RK_MPI_VDEC_CreateChn chn:%d fail %x", ctx->VdChn, ret);
        return ret;
    }

    ret = RK_MPI_VDEC_SetChnParam(ctx->VdChn, &ctx->stParam);
    if (ret) {
        RK_LOGE("RK_MPI_VDEC_SetChnParam chn:%d fail %x", ctx->VdChn, ret);
        return ret;
    }

    if (ctx->bEnableMbPool) {
        ret = RK_MPI_VDEC_AttachMbPool(ctx->VdChn, ctx->s32Pool);
        if (ret != RK_SUCCESS) {
            RK_LOGE("attatc vdec mb pool %d failed! ", ctx->VdChn);
            return ret;
        }
    }

    ret = RK_MPI_VDEC_StartRecvStream(ctx->VdChn);
    if (ret) {
        RK_LOGE("RK_MPI_VDEC_StartRecvStream chn:%d fail %x", ctx->VdChn, ret);
        return ret;
    }

	return RK_SUCCESS;
}

static RK_S32 vdec_free_buf(void *opaque) {
    if (opaque)
        free(opaque);
    return 0;
}

#define MAX_TIME_OUT_MS   20
#define BUFFER_SIZE (1024 * 1024)
RK_S32 SAMPLE_COMM_VDEC_SendStream(SAMPLE_VDEC_CTX_S *ctx, FILE *fp) {
    RK_S32 s32Ret = 0;
    RK_S32 s32ReadSize = 1024, s32Size, s32ReachEOS = 0;
    MB_EXT_CONFIG_S stMbExtConfig;
    MB_BLK buffer = RK_NULL;
    VDEC_STREAM_S stStream;
    RK_U8 *data = (RK_U8 *)malloc(s32ReadSize);;
    memset(data, 0, s32ReadSize);

__READ_AGAIN:
    s32Size = fread(data, 1, s32ReadSize, fp);
    if (s32Size <= 0) {
        // s32ReachEOS = 1;
        fseek(fp, 0, SEEK_SET);
        goto __READ_AGAIN;
    }

    memset(&stMbExtConfig, 0, sizeof(MB_EXT_CONFIG_S));
    stMbExtConfig.pFreeCB = vdec_free_buf;
    stMbExtConfig.pOpaque = data;
    stMbExtConfig.pu8VirAddr = data;
    stMbExtConfig.u64Size = s32Size;
    RK_MPI_SYS_CreateMB(&buffer, &stMbExtConfig);

    stStream.u64PTS = 0;
    stStream.pMbBlk = buffer;
    stStream.u32Len = s32Size;
    stStream.bEndOfStream = s32ReachEOS ? RK_TRUE : RK_FALSE;
    stStream.bEndOfFrame = s32ReachEOS ? RK_TRUE : RK_FALSE;
    stStream.bBypassMbBlk = RK_TRUE;

    s32Ret = RK_MPI_VDEC_SendStream(ctx->VdChn, &stStream, MAX_TIME_OUT_MS);
    if (s32Ret < 0) {
        vdec_free_buf(data);
        RK_MPI_MB_ReleaseMB(stStream.pMbBlk);
        return -1;
    } else {
        RK_MPI_MB_ReleaseMB(stStream.pMbBlk);
    }

    if (s32ReachEOS)
        RK_LOGI("chn %d input reach EOS", ctx->VdChn);

    return 0;
}

static int find_next_nal_unit(const unsigned char *buf, int size) {
    for (int i = 0; i < size - 3; i++) {
        if (buf[i] == 0x00 && buf[i + 1] == 0x00 && buf[i + 2] == 0x00 && buf[i + 3] == 0x01) {
            return i;
        }
        if (i < size - 2 && buf[i] == 0x00 && buf[i + 1] == 0x00 && buf[i + 2] == 0x01) {
            return i;
        }
    }
    return -1;
}

/* only support h264 */
RK_S32 SAMPLE_COMM_VDEC_SendFrame(SAMPLE_VDEC_CTX_S *ctx, FILE *fp, RK_U8* buffer, int *nalStart) {
    RK_S32 ret = 0;
    VDEC_STREAM_S vdec_stream;
    MB_BLK mb_blk;
    int nal_start = *nalStart;
    int nal_end;
    int last_nal_start = 0, read_size;

__reTry:
    if ((read_size = fread(buffer + nal_start, 1, BUFFER_SIZE - nal_start, fp)) <= 0) {
        fseek(fp, 0, SEEK_SET);
        nal_start = 0;
        usleep(1*1000);
        goto __reTry;
    }

    read_size += nal_start;
    nal_start = 0;
    last_nal_start = 0;

    while (1) {
        nal_end = find_next_nal_unit(buffer + nal_start, read_size - nal_start);
        if (nal_end == -1) {
            if (nal_start < read_size) {
                memmove(buffer, buffer + nal_start, read_size - nal_start);
                nal_start = read_size - nal_start;
                last_nal_start = nal_start;
            }
            break;
        }
        // printf("last_nal_start = %d, nal_start=%d, nal_end = %d  realsize = %d\n", last_nal_start, nal_start, nal_end, read_size);

        nal_end += nal_start;
        if (nal_end > nal_start) {
            RK_MPI_MMZ_Alloc(&mb_blk, nal_end - last_nal_start, 0);
            memcpy(RK_MPI_MB_Handle2VirAddr(mb_blk), buffer + last_nal_start, nal_end - last_nal_start);

            vdec_stream.pMbBlk = mb_blk;
            vdec_stream.u32Len = nal_end - last_nal_start;
            vdec_stream.u64PTS = 0;
            vdec_stream.bEndOfStream = 0;
            vdec_stream.bBypassMbBlk = RK_TRUE;
            usleep(30*1000);
            ret = RK_MPI_VDEC_SendStream(ctx->VdChn, &vdec_stream, 1000);
            if (ret)
                RK_LOGE("VDEC:%d send stream ret:%#x", ctx->VdChn, ret);
            RK_LOGD("rk_debug send frame to vdec success");
            RK_MPI_MMZ_Free(mb_blk);

            last_nal_start = nal_end;
            nal_start = nal_end;
        } else {

            if (nal_start == 0) {
                last_nal_start = 0;
            }
            nal_start++;
        }
    }

    if (nal_start < read_size) {
        RK_MPI_MMZ_Alloc(&mb_blk, read_size - nal_start, 0);
        memcpy(RK_MPI_MB_Handle2VirAddr(mb_blk), buffer + nal_start, read_size - nal_start);

        vdec_stream.pMbBlk = mb_blk;
        vdec_stream.u32Len = read_size - nal_start;
        vdec_stream.u64PTS = 0;
        vdec_stream.bEndOfStream = 1;
        vdec_stream.bBypassMbBlk = RK_TRUE;

        RK_MPI_VDEC_SendStream(ctx->VdChn, &vdec_stream, 1000);
        RK_LOGI("rk_debug send frame to vdec success");
        RK_MPI_MMZ_Free(mb_blk);
    }
    *nalStart = nal_start;
    return 0;
}

RK_S32 SAMPLE_COMM_VDEC_DestroyChn(SAMPLE_VDEC_CTX_S *ctx) {
	RK_S32 ret = RK_SUCCESS;

    ret = RK_MPI_VDEC_StopRecvStream(ctx->VdChn);
    if (ret) {
        RK_LOGE("RK_MPI_VDEC_StopRecvStream chn:%d fail %x", ctx->VdChn, ret);
        return ret;
    }

    if (ctx->bEnableMbPool && ctx->s32Pool != MB_INVALID_POOLID) {
        ret = RK_MPI_VDEC_DetachMbPool(ctx->VdChn);
        ret = RK_MPI_MB_DestroyPool(ctx->s32Pool);
        if (ret != RK_SUCCESS) {
            RK_LOGE("detatc vdec mb pool %d failed! ", ctx->VdChn);
            return ret;
        }
    }

    ret = RK_MPI_VDEC_DestroyChn(ctx->VdChn);
    if (ret) {
        RK_LOGE("RK_MPI_VDEC_DestroyChn chn:%d fail %x", ctx->VdChn, ret);
        return ret;
    }

	return RK_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
