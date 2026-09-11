#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <vector>

#include "car_recognition.h"

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

int init_car_color_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    model = read_model_data(model_path, &model_len);
    if (model == NULL)
    {
        printf("load car color model fail!\n");
        return -1;
    }

    ret = rknn_init(&ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0)
    {
        printf("rknn_init car color fail! ret=%d\n", ret);
        return -1;
    }

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("car color model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    printf("car color input tensors:\n");
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

    printf("car color output tensors:\n");
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
    printf("car color model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    free(input_attrs);
    free(output_attrs);
    return 0;
}

int release_car_color_model(rknn_app_context_t *app_ctx)
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

int inference_car_color_model(rknn_app_context_t *app_ctx, cv::Mat &car_img, car_color_result_t *color_result)
{
    int ret;
    rknn_input inputs[1];
    rknn_output outputs[1];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));

    if (car_img.empty() || car_img.cols <= 0 || car_img.rows <= 0)
    {
        return -1;
    }

    cv::Mat resized;
    cv::resize(car_img, resized, cv::Size(app_ctx->model_width, app_ctx->model_height));
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
        resized = resized - 127.5f;
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

    outputs[0].index = 0;
    outputs[0].want_float = app_ctx->is_quant ? 0 : 1;

    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        delete[] input_data_u8;
        delete[] input_data_f32;
        return ret;
    }

    int color_num = app_ctx->is_quant ? outputs[0].size : outputs[0].size / sizeof(float);
    if (color_num <= 0)
    {
        printf("Invalid car color output size: %d\n", color_num);
        rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);
        delete[] input_data_u8;
        delete[] input_data_f32;
        return -1;
    }

    std::vector<float> output_data(color_num);
    if (app_ctx->is_quant)
    {
        int8_t *output_int8 = (int8_t *)outputs[0].buf;
        float output_scale = app_ctx->output_attrs[0].scale;
        int output_zp = app_ctx->output_attrs[0].zp;
        for (int i = 0; i < color_num; i++)
        {
            output_data[i] = (output_int8[i] - output_zp) * output_scale;
        }
    }
    else
    {
        float *output_f32 = (float *)outputs[0].buf;
        for (int i = 0; i < color_num; i++)
        {
            output_data[i] = output_f32[i];
        }
    }

    std::vector<float> probs(color_num);
    float sum = 0;
    for (int i = 0; i < color_num; i++)
    {
        probs[i] = exp(output_data[i]);
        sum += probs[i];
    }
    for (int i = 0; i < color_num; i++)
    {
        probs[i] /= sum;
    }

    int max_idx = 0;
    float max_prob = probs[0];
    for (int i = 1; i < color_num; i++)
    {
        if (probs[i] > max_prob)
        {
            max_prob = probs[i];
            max_idx = i;
        }
    }

    int second_idx = max_idx == 0 ? 1 : 0;
    for (int i = 0; i < color_num; i++)
    {
        if (i != max_idx && probs[i] > probs[second_idx])
        {
            second_idx = i;
        }
    }

    if (max_prob < 0.80f && second_idx >= 0 && second_idx < color_num)
    {
        const char *best_cn = max_idx < static_cast<int>(car_colors.size()) ? car_colors[max_idx].c_str() : "";
        const char *second_cn = second_idx < static_cast<int>(car_colors.size()) ? car_colors[second_idx].c_str() : "";
        printf("Car color top2: %s=%.3f, %s=%.3f\n",
               best_cn, max_prob, second_cn, probs[second_idx]);
    }

    color_result->color_id = max_idx;
    color_result->confidence = max_prob;
    if (max_idx >= 0 && max_idx < static_cast<int>(car_colors.size()))
    {
        color_result->color_name = car_colors[max_idx];
    }
    else
    {
        // max_idx 超出 car_colors 列表 -> 模型类别数与列表不匹配, 置空避免读到未初始化内存
        color_result->color_name.clear();
        printf("[CAR-COLOR] WARN max_idx=%d 超出 car_colors(%zu) 类别, 类别数不匹配?\n",
               max_idx, car_colors.size());
    }

    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);
    delete[] input_data_u8;
    delete[] input_data_f32;

    return 0;
}
