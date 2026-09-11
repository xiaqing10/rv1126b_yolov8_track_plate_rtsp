#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <algorithm>
#include <vector>
#include <string>

#include "car_recognition.h"

// 保存车牌识别网络的输入图(实际送入模型的裁剪图), 文件名为 模型_识别结果_置信度, 便于核对准确性。
// 目录可用环境变量 PLATE_SAVE_DIR 覆盖, 默认 /tmp/plate_imgs。
static void lprnet_save_input(const cv::Mat &img, const std::string &plate, float conf)
{
    const char *dir = getenv("PLATE_SAVE_DIR");
    if (!dir || !*dir) dir = "/tmp/plate_imgs";
    mkdir(dir, 0755);

    char fname[256];
    snprintf(fname, sizeof(fname), "%s/lprnet_%s_%.0f%%.jpg", dir, plate.c_str(), conf * 100);
    // 去掉文件名里可能不合法的字符(空格/斜杠等)
    for (char *p = fname; *p; p++)
        if (*p == ' ' || *p == '/' || *p == '\\' || *p == ':') *p = '_';
    cv::imwrite(fname, img);
}

// LPRNet 字符集 (68 类, 最后一个为 blank)
const std::vector<std::string>
    lprnet_plate_code{
        "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
        "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
        "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁",
        "新",
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
        "A", "B", "C", "D", "E", "F", "G", "H", "J", "K",
        "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "I", "O", "-"};

static void lprnet_dump_tensor_attr(rknn_tensor_attr *attr)
{
    printf("  index=%d, name=%s, n_dims=%d, n_elems=%d, size=%d, fmt=%d, type=%d, qnt_type=%d, zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->n_elems, attr->size,
           attr->fmt, attr->type, attr->qnt_type, attr->zp, attr->scale);
}

static char *lprnet_read_model(const char *filename, int *model_size)
{
    FILE *fp = fopen(filename, "rb");
    if (fp == nullptr)
    {
        printf("fopen %s fail!\n", filename);
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    char *data = (char *)malloc(size);
    if (data == nullptr)
    {
        fclose(fp);
        return NULL;
    }
    fseek(fp, 0, SEEK_SET);
    if (fread(data, 1, size, fp) != (size_t)size)
    {
        free(data);
        fclose(fp);
        return NULL;
    }
    fclose(fp);
    *model_size = size;
    return data;
}

int init_lprnet_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    model = lprnet_read_model(model_path, &model_len);
    if (model == NULL)
    {
        printf("load lprnet model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init lprnet fail! ret=%d\n", ret);
        return -1;
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("lprnet model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    printf("lprnet input tensors:\n");
    rknn_tensor_attr *input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    if (input_attrs == NULL)
    {
        printf("malloc input_attrs failed!\n");
        return -1;
    }
    memset(input_attrs, 0, io_num.n_input * sizeof(rknn_tensor_attr));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            free(input_attrs);
            return -1;
        }
        lprnet_dump_tensor_attr(&(input_attrs[i]));
    }

    printf("lprnet output tensors:\n");
    rknn_tensor_attr *output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    if (output_attrs == NULL)
    {
        printf("malloc output_attrs failed!\n");
        free(input_attrs);
        return -1;
    }
    memset(output_attrs, 0, io_num.n_output * sizeof(rknn_tensor_attr));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            free(input_attrs);
            free(output_attrs);
            return -1;
        }
        lprnet_dump_tensor_attr(&(output_attrs[i]));
    }

    app_ctx->rknn_ctx = ctx;
    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
        output_attrs[0].type != RKNN_TENSOR_FLOAT16)
    {
        app_ctx->is_quant = true;
    }
    else
    {
        app_ctx->is_quant = false;
    }

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height = input_attrs[0].dims[2];
        app_ctx->model_width = input_attrs[0].dims[3];
    }
    else
    {
        app_ctx->model_height = input_attrs[0].dims[1];
        app_ctx->model_width = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }
    printf("lprnet model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    free(input_attrs);
    free(output_attrs);
    return 0;
}

int release_lprnet_model(rknn_app_context_t *app_ctx)
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

int inference_lprnet_model(rknn_app_context_t *app_ctx, cv::Mat &plate_img, std::string *plate_number)
{
    int ret;
    rknn_input inputs[1];
    rknn_output outputs[1];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    if (plate_img.empty() || plate_img.cols <= 0 || plate_img.rows <= 0)
    {
        return -1;
    }

    // 输入: 94x24 BGR uint8 (与 LPRNet Python demo 一致, 不做通道交换/归一化)
    cv::Mat resized;
    cv::resize(plate_img, resized, cv::Size(LPRNET_MODEL_WIDTH, LPRNET_MODEL_HEIGHT));
    if (resized.channels() != 3)
    {
        cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
    }

    int input_size = LPRNET_MODEL_WIDTH * LPRNET_MODEL_HEIGHT * 3;
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = input_size;
    inputs[0].buf = resized.data;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, 1, inputs);
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        return -1;
    }

    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        return -1;
    }

    outputs[0].index = 0;
    outputs[0].want_float = 1;
    ret = rknn_outputs_get(app_ctx->rknn_ctx, 1, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        return ret;
    }

    // 输出布局 [OUT_ROWS, OUT_COLS], 对每个位置 x 取 68 类的 argmax
    // 置信度用"每位置 68 类 softmax 后的最大概率"再求平均, 得到 [0,1] 的可解释准确率。
    float *ptr = (float *)outputs[0].buf;
    std::vector<int> prebs(LPRNET_OUT_COLS);
    float conf_sum = 0.0f;
    for (int x = 0; x < LPRNET_OUT_COLS; x++)
    {
        float *preb = ptr + x;              // 步长 OUT_COLS
        int max_idx = 0;
        float max_val = preb[0];
        // 稳定 softmax: 先取该位置 68 类的最大值
        float max_l = preb[0];
        for (int y = 1; y < LPRNET_OUT_ROWS; y++)
            if (preb[y * LPRNET_OUT_COLS] > max_l)
                max_l = preb[y * LPRNET_OUT_COLS];
        double sum = 0.0;
        for (int y = 0; y < LPRNET_OUT_ROWS; y++)
        {
            double e = exp((double)preb[y * LPRNET_OUT_COLS] - (double)max_l);
            sum += e;
            if (preb[y * LPRNET_OUT_COLS] > max_val)
            {
                max_val = preb[y * LPRNET_OUT_COLS];
                max_idx = y;
            }
        }
        prebs[x] = max_idx;
        // 最大类的 softmax 概率
        conf_sum += (float)(exp((double)max_val - (double)max_l) / sum);
    }
    float avg_conf = conf_sum / LPRNET_OUT_COLS;

    // CTC 去重去 blank
    const int blank = LPRNET_OUT_ROWS - 1;
    std::string out;
    int pre_c = blank;
    for (int x = 0; x < LPRNET_OUT_COLS; x++)
    {
        int c = prebs[x];
        if (c == blank || c == pre_c)
        {
            if (c == blank)
                pre_c = blank;
            continue;
        }
        if (c >= 0 && c < (int)lprnet_plate_code.size())
            out += lprnet_plate_code[c];
        pre_c = c;
    }

    // 保存进入网络的输入图, 文件名=识别内容+置信度, 便于核对识别准确性
    lprnet_save_input(resized, out, avg_conf);
    printf("[LPRNET] decoded plate: %s, avg_conf=%.3f, img=%dx%d\n",
           out.c_str(), avg_conf, resized.cols, resized.rows);

    if (plate_number)
        *plate_number = out;

    rknn_outputs_release(app_ctx->rknn_ctx, 1, outputs);
    return 0;
}
