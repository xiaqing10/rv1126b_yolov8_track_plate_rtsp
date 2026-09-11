// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "yolov8.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"
#include "npu_lock.h"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

int init_yolov8_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    // Load RKNN Model
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL)
    {
        printf("load_model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    // Get Model Input Output Number
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // Get Model Input Info
    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // Get Model Output Info
    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(output_attrs[i]));
    }

    // Set to context
    app_ctx->rknn_ctx = ctx;

    // TODO
    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type == RKNN_TENSOR_INT8)
    {
        app_ctx->is_quant = true;
    }
    else
    {
        app_ctx->is_quant = false;
    }

    // 自动推断类别数: YOLOv8 9 输出时 score 分支通道 = dims[1](NCHW);
    // output_attrs 复制到 app_ctx 后, 用与 post_process 一致的索引读取。
    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (g_obj_class_num <= 0) {
        int output_per_branch = io_num.n_output / 3;
        int score_idx = output_per_branch + 1;          // 第一组的 score 分支
        if (io_num.n_output >= 3 && score_idx < (int)io_num.n_output) {
            g_obj_class_num = app_ctx->output_attrs[score_idx].dims[1];  // NCHW: [1, cls, gh, gw]
        }
        if (g_obj_class_num <= 0 || g_obj_class_num > OBJ_CLASS_MAX) {
            printf("WARN: cannot infer class num from model, fallback to 80\n");
            g_obj_class_num = 80;
        }
    }
    printf("detect class num = %d\n", g_obj_class_num);

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        app_ctx->model_height = input_attrs[0].dims[1];
        app_ctx->model_width = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    return 0;
}

int release_yolov8_model(rknn_app_context_t *app_ctx)
{
    if (app_ctx->input_attrs != NULL)
    {
        free(app_ctx->input_attrs);
        app_ctx->input_attrs = NULL;
    }
    if (app_ctx->output_attrs != NULL)
    {
        free(app_ctx->output_attrs);
        app_ctx->output_attrs = NULL;
    }
    if (app_ctx->rknn_ctx != 0)
    {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

// ================= 三段式流水线接口 =================
// 为支持 预处理/推理(NPU)/后处理 三线程重叠, 把原来的串行推理拆成 3 个可独立调用的阶段。
// 调用方负责: 为 dst_img 分配/复用 virt_addr, 保存 outputs 直到后处理完成后 release。

// 阶段1: 预处理 (RGB -> 640x640 letterbox)。dst_img->virt_addr 若为 NULL 会自行分配。
int yolov8_preprocess(rknn_app_context_t *app_ctx, image_buffer_t *img,
                      image_buffer_t *dst_img, letterbox_t *letter_box)
{
    if ((!app_ctx) || (!img) || (!dst_img)) return -1;
    // 注意: 不 memset 整个结构, 以保留调用方预先分配的 virt_addr(流水线槽位复用)。
    dst_img->width = app_ctx->model_width;
    dst_img->height = app_ctx->model_height;
    dst_img->format = IMAGE_FORMAT_RGB888;
    dst_img->size = get_image_size(dst_img);
    if (!dst_img->virt_addr) {
        dst_img->virt_addr = (unsigned char *)malloc(dst_img->size);
        if (!dst_img->virt_addr) { printf("malloc letterbox buf %d fail\n", dst_img->size); return -1; }
    }
    return convert_image_with_letterbox(img, dst_img, letter_box, 114);
}

// 阶段2: NPU 推理 (inputs_set + rknn_run + outputs_get)。
// 注意: rknn context 非线程安全, 本函数必须只在同一线程调用。
int yolov8_infer(rknn_app_context_t *app_ctx, image_buffer_t *dst_img, rknn_output *outputs)
{
    rknn_input inputs[app_ctx->io_num.n_input];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = dst_img->virt_addr;

    int ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    if (ret < 0) { printf("rknn_input_set fail! ret=%d\n", ret); return ret; }
    // 串行化 NPU 会话: 与 ReID reid_rknn.cpp 共用 g_npu_lock, 避免与 ReID worker
    // 并发 rknn_run 导致 NPU 驱动堆损坏 (跨线程 Eigen free 段错误根因)。
    pthread_mutex_lock(&npu_lock());
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0) { printf("rknn_run fail! ret=%d\n", ret); pthread_mutex_unlock(&npu_lock()); return ret; }

    memset(outputs, 0, app_ctx->io_num.n_output * sizeof(rknn_output));
    for (int i = 0; i < app_ctx->io_num.n_output; i++) {
        outputs[i].index = i;
        outputs[i].want_float = (!app_ctx->is_quant);
    }
    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0) { printf("rknn_outputs_get fail! ret=%d\n", ret); pthread_mutex_unlock(&npu_lock()); return ret; }
    pthread_mutex_unlock(&npu_lock());
    return 0;
}

// 阶段3: 后处理 (outputs -> od_results)。用完 outputs 后必须由调用方 rknn_outputs_release。
int yolov8_postprocess(rknn_app_context_t *app_ctx, rknn_output *outputs,
                       letterbox_t *letter_box, object_detect_result_list *od_results)
{
    if ((!app_ctx) || (!outputs) || (!od_results)) return -1;
    memset(od_results, 0x00, sizeof(*od_results));
    return post_process(app_ctx, outputs, letter_box, g_box_conf_thresh, g_nms_thresh, od_results);
}

int inference_yolov8_model(rknn_app_context_t *app_ctx, image_buffer_t *img, object_detect_result_list *od_results)
{
    if ((!app_ctx) || !(img) || (!od_results)) return -1;

    image_buffer_t dst_img;
    letterbox_t letter_box;
    rknn_output outputs[app_ctx->io_num.n_output];
    memset(&dst_img, 0, sizeof(dst_img));
    memset(&letter_box, 0, sizeof(letterbox_t));

    double t_lb0 = now_ms();
    int ret = yolov8_preprocess(app_ctx, img, &dst_img, &letter_box);
    double t_lb1 = now_ms();
    if (ret < 0) { printf("preprocess fail! ret=%d\n", ret); return -1; }

    double t_is0 = now_ms();
    ret = yolov8_infer(app_ctx, &dst_img, outputs);
    double t_og1 = now_ms();
    if (ret < 0) { printf("infer fail! ret=%d\n", ret); goto out; }

    {
        double t_pp0 = now_ms();
        ret = yolov8_postprocess(app_ctx, outputs, &letter_box, od_results);
        double t_pp1 = now_ms();
        printf("[timing] letterbox=%.1f infer=%d=%.1f post=%d=%.1f total=%.1f ms\n",
               t_lb1 - t_lb0, od_results->count, t_og1 - t_is0,
               od_results->count, t_pp1 - t_pp0, t_pp1 - t_lb0);
    }

    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

out:
    if (dst_img.virt_addr != NULL) free(dst_img.virt_addr);
    return ret;
}