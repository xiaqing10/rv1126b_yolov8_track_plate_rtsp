#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include <algorithm>
#include <vector>
#include <numeric>

#include "car_recognition.h"

// 保存车牌识别网络的输入图(实际送入模型的裁剪图), 文件名为识别结果+置信度, 便于核对准确性。
// 目录可用环境变量 PLATE_SAVE_DIR 覆盖, 默认 /tmp/plate_imgs。
static void plate_rec_save_input(const cv::Mat &img, const std::string &plate, float conf)
{
    const char *dir = getenv("PLATE_SAVE_DIR");
    if (!dir || !*dir) dir = "/tmp/plate_imgs";
    mkdir(dir, 0755);

    char fname[256];
    snprintf(fname, sizeof(fname), "%s/plate_rec_%s_%.0f%%.jpg", dir, plate.c_str(), conf * 100);
    for (char *p = fname; *p; p++)
        if (*p == ' ' || *p == '/' || *p == '\\' || *p == ':') *p = '_';
    cv::imwrite(fname, img);
}

typedef struct
{
    std::string plate_number;
    std::string plate_color;
    int color_id;
    float avg_confidence;
    int char_count;
} plate_decode_candidate_t;

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; ++i)
    {
        shape_str += ", " + std::to_string(attr->dims[i]);
    }
    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, fmt=%d, type=%d, qnt_type=%d, zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size,
           attr->fmt, attr->type, attr->qnt_type, attr->zp, attr->scale);
}

static char *read_model_data(const char *filename, int *model_size)
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
    if (fread(data, 1, size, fp) != size)
    {
        free(data);
        fclose(fp);
        return NULL;
    }
    fclose(fp);
    *model_size = size;
    return data;
}

static int utf8_codepoint_count(const std::string &text)
{
    int count = 0;
    for (unsigned char ch : text)
    {
        if ((ch & 0xC0) != 0x80)
        {
            count++;
        }
    }
    return count;
}

static cv::Mat add_plate_padding(const cv::Mat &plate_img)
{
    if (plate_img.empty())
    {
        return plate_img.clone();
    }

    int pad_x = std::max(2, static_cast<int>(std::round(plate_img.cols * 0.08f)));
    int pad_y = std::max(2, static_cast<int>(std::round(plate_img.rows * 0.18f)));

    cv::Mat padded;
    cv::copyMakeBorder(plate_img, padded, pad_y, pad_y, pad_x, pad_x, cv::BORDER_REPLICATE);
    return padded;
}

static bool is_better_plate_candidate(const plate_decode_candidate_t &candidate,
                                      const plate_decode_candidate_t &best)
{
    if (candidate.char_count != best.char_count)
    {
        return candidate.char_count > best.char_count;
    }
    return candidate.avg_confidence > best.avg_confidence + 1e-4f;
}

static int run_plate_rec_once(rknn_app_context_t *app_ctx, const cv::Mat &plate_img,
                              plate_decode_candidate_t *candidate, bool verbose_log)
{
    int ret;
    rknn_input inputs[1];
    rknn_output outputs[2];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    cv::Mat resized;
    cv::resize(plate_img, resized, cv::Size(app_ctx->model_width, app_ctx->model_height));
    cv::Mat save_img = resized.clone();   // BGR 输入图备份, 用于保存核对
    int input_size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].index = 0;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = input_size;

    unsigned char *input_data_u8 = nullptr;
    float *input_data_f32 = nullptr;
    if (app_ctx->is_quant)
    {
        inputs[0].type = RKNN_TENSOR_UINT8;
        input_data_u8 = new unsigned char[input_size];
        memcpy(input_data_u8, resized.data, input_size);
        inputs[0].buf = input_data_u8;
    }
    else
    {
        resized.convertTo(resized, CV_32F);
        float mean = 0.588f;
        float std = 0.193f;
        resized = (resized / 255.0f - mean) / std;

        input_data_f32 = new float[input_size];
        memcpy(input_data_f32, resized.data, input_size * sizeof(float));
        inputs[0].type = RKNN_TENSOR_FLOAT32;
        inputs[0].size = input_size * sizeof(float);
        inputs[0].buf = input_data_f32;
    }

    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
    if (ret < 0)
    {
        printf("rknn_input_set fail! ret=%d\n", ret);
        delete[] input_data_u8;
        delete[] input_data_f32;
        return -1;
    }

    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0)
    {
        printf("rknn_run fail! ret=%d\n", ret);
        delete[] input_data_u8;
        delete[] input_data_f32;
        return -1;
    }

    for (int i = 0; i < app_ctx->io_num.n_output; i++)
    {
        outputs[i].index = i;
        outputs[i].want_float = app_ctx->is_quant ? 0 : 1;
    }

    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        delete[] input_data_u8;
        delete[] input_data_f32;
        return ret;
    }

    int plate_chars_num = static_cast<int>(plate_chars.size());
    int plate_elem_count = app_ctx->is_quant ? outputs[0].size : outputs[0].size / sizeof(float);
    if (plate_elem_count <= 0 || plate_elem_count % plate_chars_num != 0)
    {
        printf("Invalid plate output size: %d, chars_num=%d\n", plate_elem_count, plate_chars_num);
        rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);
        delete[] input_data_u8;
        delete[] input_data_f32;
        return -1;
    }
    int plate_seq_len = plate_elem_count / plate_chars_num;

    std::vector<float> plate_output(plate_elem_count);
    if (app_ctx->is_quant)
    {
        int8_t *plate_output_int8 = (int8_t *)outputs[0].buf;
        float plate_scale = app_ctx->output_attrs[0].scale;
        int plate_zp = app_ctx->output_attrs[0].zp;
        for (int i = 0; i < plate_elem_count; i++)
        {
            plate_output[i] = (plate_output_int8[i] - plate_zp) * plate_scale;
        }
        // 诊断: 量化输出统计, 排查近均匀(置信度过低)问题
        float mn = plate_output[0], mx = plate_output[0];
        for (int i = 1; i < plate_elem_count; i++)
        {
            if (plate_output[i] < mn) mn = plate_output[i];
            if (plate_output[i] > mx) mx = plate_output[i];
        }
        printf("[REC-Q] quant scale=%f zp=%d out_elems=%d min=%.4f max=%.4f\n",
               plate_scale, plate_zp, plate_elem_count, mn, mx);
    }
    else
    {
        float *plate_output_f32 = (float *)outputs[0].buf;
        for (int i = 0; i < plate_elem_count; i++)
        {
            plate_output[i] = plate_output_f32[i];
        }
        float mn = plate_output[0], mx = plate_output[0];
        for (int i = 1; i < plate_elem_count; i++)
        {
            if (plate_output[i] < mn) mn = plate_output[i];
            if (plate_output[i] > mx) mx = plate_output[i];
        }
        printf("[REC-F] out_elems=%d min=%.4f max=%.4f\n", plate_elem_count, mn, mx);
    }

    // 诊断: 每行(每时刻)原始 logit 的 top2 之差(margin)。margin 大 -> softmax 应尖锐(高分);
    //        若各位置 margin 都极小(分布近均匀) -> 输入让模型没把握(输入质量/格式问题)。
    for (int i = 0; i < plate_seq_len && i < 8; i++)
    {
        const float *row = plate_output.data() + i * plate_chars_num;
        int top1 = 0, top2 = 1;
        float v1 = row[0], v2 = row[1];
        if (v2 > v1) { std::swap(v1, v2); top1 = 1; top2 = 0; }
        for (int c = 2; c < plate_chars_num; c++)
        {
            if (row[c] > v1) { v2 = v1; top2 = top1; v1 = row[c]; top1 = c; }
            else if (row[c] > v2) { v2 = row[c]; top2 = c; }
        }
        printf("[REC-M%02d] top1=%s(%.3f) top2=%s(%.3f) margin=%.3f\n",
               i, plate_chars[top1].c_str(), v1, plate_chars[top2].c_str(), v2, v1 - v2);
    }

    for (int i = 0; i < plate_seq_len; i++)
    {
        float *row = plate_output.data() + i * plate_chars_num;
        float max_val = row[0];
        for (int c = 1; c < plate_chars_num; c++)
        {
            if (row[c] > max_val)
            {
                max_val = row[c];
            }
        }
        float sum = 0.0f;
        for (int c = 0; c < plate_chars_num; c++)
        {
            row[c] = expf(row[c] - max_val);
            sum += row[c];
        }
        for (int c = 0; c < plate_chars_num; c++)
        {
            row[c] /= sum;
        }
    }

    std::vector<int> plate_indices(plate_seq_len);
    std::vector<float> timestep_conf(plate_seq_len, 0.0f);
    for (int i = 0; i < plate_seq_len; i++)
    {
        int max_idx = 0;
        float max_val = plate_output[i * plate_chars_num];
        for (int c = 1; c < plate_chars_num; c++)
        {
            float v = plate_output[i * plate_chars_num + c];
            if (v > max_val)
            {
                max_val = v;
                max_idx = c;
            }
        }
        // plate_output 已被上面第一个循环 softmax 成 [0,1] 概率, 最大值即该时刻置信度。
        // (旧实现在这里又对概率做了一次 softmax, 导致置信度被除以 ~类别数 而塌陷成 ~3%。)
        plate_indices[i] = max_idx;
        timestep_conf[i] = max_val;
    }

    std::string plate_number;
    float conf_sum = 0.0f;
    int conf_count = 0;
    int prev = -1;
    for (int i = 0; i < plate_seq_len; i++)
    {
        int idx = plate_indices[i];
        if (idx != 0 && idx != prev)
        {
            if (idx >= 0 && idx < static_cast<int>(plate_chars.size()))
            {
                plate_number += plate_chars[idx];
                conf_sum += timestep_conf[i];
                conf_count++;
            }
        }
        prev = idx;
    }

    int color_id = 0;
    if (app_ctx->io_num.n_output >= 2)
    {
        int color_num = app_ctx->is_quant ? outputs[1].size : outputs[1].size / sizeof(float);
        if (color_num > 0)
        {
            std::vector<float> color_output(color_num);
            if (app_ctx->is_quant)
            {
                int8_t *color_output_int8 = (int8_t *)outputs[1].buf;
                float color_scale = app_ctx->output_attrs[1].scale;
                int color_zp = app_ctx->output_attrs[1].zp;
                for (int i = 0; i < color_num; i++)
                {
                    color_output[i] = (color_output_int8[i] - color_zp) * color_scale;
                }
            }
            else
            {
                float *color_output_f32 = (float *)outputs[1].buf;
                for (int i = 0; i < color_num; i++)
                {
                    color_output[i] = color_output_f32[i];
                }
            }

            float max_color = color_output[0];
            for (int i = 1; i < color_num; i++)
            {
                if (color_output[i] > max_color)
                {
                    max_color = color_output[i];
                    color_id = i;
                }
            }
        }
    }

    candidate->plate_number = plate_number;
    candidate->color_id = color_id;
    candidate->plate_color.clear();
    if (color_id >= 0 && color_id < static_cast<int>(plate_colors.size()))
    {
        candidate->plate_color = plate_colors[color_id];
    }
    candidate->avg_confidence = conf_count > 0 ? conf_sum / conf_count : 0.0f;
    candidate->char_count = utf8_codepoint_count(plate_number);

    if (verbose_log)
    {
        printf("Plate indices: ");
    for (int i = 0; i < plate_seq_len; i++)
        {
            printf("%d ", plate_indices[i]);
        }
        printf("\nPlate crop size: %dx%d, decoded plate: %s, avg_conf=%.3f\n",
               plate_img.cols, plate_img.rows, plate_number.c_str(), candidate->avg_confidence);
    }

    if (verbose_log)
    {
        plate_rec_save_input(save_img, plate_number, candidate->avg_confidence);
    }

    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);
    delete[] input_data_u8;
    delete[] input_data_f32;
    return 0;
}

int init_plate_rec_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    model = read_model_data(model_path, &model_len);
    if (model == NULL)
    {
        printf("load plate rec model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init plate rec fail! ret=%d\n", ret);
        return -1;
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("plate rec model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    printf("plate rec input tensors:\n");
    rknn_tensor_attr *input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    if (input_attrs == NULL) {
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
        dump_tensor_attr(&(input_attrs[i]));
    }

    printf("plate rec output tensors:\n");
    rknn_tensor_attr *output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    if (output_attrs == NULL) {
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
        dump_tensor_attr(&(output_attrs[i]));
    }

    app_ctx->rknn_ctx = ctx;
    app_ctx->io_num = io_num;
    app_ctx->input_attrs = (rknn_tensor_attr *)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->input_attrs, input_attrs, io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr *)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    memcpy(app_ctx->output_attrs, output_attrs, io_num.n_output * sizeof(rknn_tensor_attr));

    if (output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC && output_attrs[0].type != RKNN_TENSOR_FLOAT16)
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
    printf("plate rec model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    free(input_attrs);
    free(output_attrs);
    return 0;
}

int release_plate_rec_model(rknn_app_context_t *app_ctx)
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

int inference_plate_rec_model(rknn_app_context_t *app_ctx, cv::Mat &plate_img, plate_result_t *plate_result)
{
    plate_decode_candidate_t best_candidate = {};
    int ret = run_plate_rec_once(app_ctx, plate_img, &best_candidate, true);
    if (ret != 0)
    {
        return ret;
    }

    if (best_candidate.char_count < 7)
    {
        cv::Mat padded_plate = add_plate_padding(plate_img);
        if (!padded_plate.empty() &&
            (padded_plate.cols != plate_img.cols || padded_plate.rows != plate_img.rows))
        {
            plate_decode_candidate_t retry_candidate = {};
            if (run_plate_rec_once(app_ctx, padded_plate, &retry_candidate, false) == 0)
            {
                printf("Plate retry with padding: %dx%d -> %dx%d, decoded plate: %s, avg_conf=%.3f\n",
                       plate_img.cols, plate_img.rows, padded_plate.cols, padded_plate.rows,
                       retry_candidate.plate_number.c_str(), retry_candidate.avg_confidence);
                if (is_better_plate_candidate(retry_candidate, best_candidate))
                {
                    best_candidate = retry_candidate;
                }
            }
        }
    }

    plate_result->plate_number = best_candidate.plate_number;
    plate_result->plate_color = best_candidate.plate_color;
    plate_result->color_id = best_candidate.color_id;
    return 0;
}
