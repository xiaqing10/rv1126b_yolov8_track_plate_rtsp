#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <sstream>
#include <vector>

#include "car_recognition.h"

static inline float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

static std::vector<cv::Point2f> order_points(const std::vector<cv::Point2f> &points)
{
    if (points.size() != 4)
    {
        return points;
    }

    std::vector<cv::Point2f> rect(4);
    std::vector<float> sums(4), diffs(4);
    for (size_t i = 0; i < points.size(); ++i)
    {
        sums[i] = points[i].x + points[i].y;
        diffs[i] = points[i].y - points[i].x;
    }

    rect[0] = points[std::min_element(sums.begin(), sums.end()) - sums.begin()];
    rect[2] = points[std::max_element(sums.begin(), sums.end()) - sums.begin()];
    rect[1] = points[std::min_element(diffs.begin(), diffs.end()) - diffs.begin()];
    rect[3] = points[std::max_element(diffs.begin(), diffs.end()) - diffs.begin()];
    return rect;
}

// 78 characters total: 1 blank + 31 provinces + 10 special + 10 digits + 24 letters + 2 special chars
const std::vector<std::string> plate_chars = {
    // Index 0: blank/unknown
    "#",
    // Index 1-31: Province abbreviations (31 provinces)
    "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
    "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
    "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁",
    "新",
    // Index 32-41: Special characters (10)
    "学", "警", "港", "澳", "挂", "使", "领", "民", "航", "危",
    // Index 42-51: Digits (10)
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    // Index 52-75: Letters (24, excluding I and O)
    "A", "B", "C", "D", "E", "F", "G", "H", "J", "K",
    "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V",
    "W", "X", "Y", "Z",
    // Index 76-77: Additional special characters (2)
    "险", "品"
};

const std::vector<std::string> plate_colors = {
    "Black", "Blue", "Green", "White", "Yellow"
};

const std::vector<std::string> car_colors = {
    "Black", "Blue", "Yellow", "Brown", "Green", "Gray", "Orange", "Pink", "Purple", "Red", "White"
};

const std::vector<std::string> class_names = {
    "SinglePlate", "DoublePlate", "Car"
};

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

static float iou(const detect_box_t &a, const detect_box_t &b)
{
    float x1 = std::max(a.x1, b.x1);
    float y1 = std::max(a.y1, b.y1);
    float x2 = std::min(a.x2, b.x2);
    float y2 = std::min(a.y2, b.y2);
    
    float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    float union_area = area_a + area_b - intersection;
    
    return intersection / (union_area + 1e-6f);
}

static std::vector<int> nms(std::vector<detect_box_t> &boxes, float iou_threshold)
{
    std::vector<int> indices(boxes.size());
    for (size_t i = 0; i < boxes.size(); i++)
    {
        indices[i] = i;
    }
    
    std::sort(indices.begin(), indices.end(), [&boxes](int i, int j) {
        return boxes[i].conf > boxes[j].conf;
    });
    
    std::vector<int> keep;
    std::vector<bool> suppressed(boxes.size(), false);
    
    for (int i : indices)
    {
        if (suppressed[i])
            continue;
        keep.push_back(i);
        
        for (int j : indices)
        {
            if (suppressed[j])
                continue;
            if (boxes[i].cls_id == boxes[j].cls_id && iou(boxes[i], boxes[j]) > iou_threshold)
            {
                suppressed[j] = true;
            }
        }
    }
    
    return keep;
}

// 跨类别 NMS: 仅在车牌类(0/1)之间合并重叠框, 保留置信度高的一个。
// 汽车类(2)包含车牌, 不做跨类别抑制。返回仍需保留的索引。
static std::vector<int> cross_class_plate_nms(const std::vector<detect_box_t> &boxes,
                                              const std::vector<int> &keep_in,
                                              float iou_threshold)
{
    std::vector<int> plates;
    for (int idx : keep_in)
        if (boxes[idx].cls_id == 0 || boxes[idx].cls_id == 1)
            plates.push_back(idx);

    std::vector<int> result;
    for (size_t i = 0; i < plates.size(); i++)
    {
        bool suppressed = false;
        for (size_t j = 0; j < i; j++)
        {
            // plates 按置信度降序 (来自 nms 的排序), 高置信度优先保留
            if (iou(boxes[plates[j]], boxes[plates[i]]) > iou_threshold)
            {
                suppressed = true;
                break;
            }
        }
        if (!suppressed)
            result.push_back(plates[i]);
    }
    return result;
}

int init_detect_model(const char *model_path, rknn_app_context_t *app_ctx)
{
    int ret;
    int model_len = 0;
    char *model;
    rknn_context ctx = 0;

    model = read_model_data(model_path, &model_len);
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

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
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
    }

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

    // 从输入张量属性解析模型宽/高/通道 (NCHW 或 NHWC)
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        app_ctx->model_channel = input_attrs[0].dims[1];
        app_ctx->model_height  = input_attrs[0].dims[2];
        app_ctx->model_width   = input_attrs[0].dims[3];
    }
    else // NHWC
    {
        app_ctx->model_height  = input_attrs[0].dims[1];
        app_ctx->model_width   = input_attrs[0].dims[2];
        app_ctx->model_channel = input_attrs[0].dims[3];
    }

    printf("detect model input height=%d, width=%d, channel=%d\n",
           app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

    free(input_attrs);
    free(output_attrs);
    return 0;
}

int release_detect_model(rknn_app_context_t *app_ctx)
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

int inference_detect_model(rknn_app_context_t *app_ctx, cv::Mat &img, detect_result_list_t *det_results,
                           float conf_threshold, float nms_threshold, float plate_nms_threshold)
{
    int ret;
    rknn_input inputs[1];
    rknn_output outputs[1];

    memset(inputs, 0, sizeof(inputs));
    memset(outputs, 0, sizeof(outputs));
    memset(det_results, 0, sizeof(detect_result_list_t));

    if (img.empty())
    {
        printf("Input image is empty!\n");
        return -1;
    }

    cv::Mat letterboxed = letterbox(img, app_ctx->model_width, cv::Scalar(114, 114, 114));
    if (letterboxed.empty())
    {
        printf("Letterbox failed!\n");
        return -1;
    }

    cv::Mat rgb;
    cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);
    if (rgb.empty())
    {
        printf("Color conversion failed!\n");
        return -1;
    }

    int img_width = img.cols;
    int img_height = img.rows;
    float gain = static_cast<float>(app_ctx->model_width) / std::max(img_width, img_height);
    float pad_w = (app_ctx->model_width - img_width * gain) / 2;
    float pad_h = (app_ctx->model_height - img_height * gain) / 2;

    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].size = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
    inputs[0].buf = rgb.data;

    ret = rknn_inputs_set(app_ctx->rknn_ctx, app_ctx->io_num.n_input, inputs);
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

    outputs[0].want_float = 0;
    ret = rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
    if (ret < 0)
    {
        printf("rknn_outputs_get fail! ret=%d\n", ret);
        return ret;
    }

    int8_t *output_int8 = (int8_t *)outputs[0].buf;
    int output_size = outputs[0].size;
    
    float output_scale = app_ctx->output_attrs[0].scale;
    int output_zp = app_ctx->output_attrs[0].zp;
    
    int num_anchors = output_size / 16;
    if (num_anchors <= 0 || num_anchors > 100000)
    {
        printf("Invalid num_anchors: %d, output_size: %d\n", num_anchors, output_size);
        return -1;
    }

    float *output_data = new float[output_size];
    for (int i = 0; i < output_size; i++)
    {
        output_data[i] = (output_int8[i] - output_zp) * output_scale;
    }

    std::vector<detect_box_t> boxes;
    std::vector<std::vector<landmark_point_t>> all_landmarks;
    std::vector<int> class_ids;
    std::vector<float> confidences;

    for (int i = 0; i < num_anchors; i++)
    {
        const float *ptr = output_data + i * 16;
        
        if (i * 16 + 14 >= output_size)
        {
            printf("Warning: index %d out of bounds\n", i * 16 + 14);
            break;
        }
        
        float cx = ptr[0];
        float cy = ptr[1];
        float w = ptr[2];
        float h = ptr[3];
        float obj_conf = sigmoid(ptr[4]);

        int class_id = 0;
        float class_conf = 0.0f;
        
        for (int c = 0; c < 3; c++)
        {
            float cls_conf = sigmoid(ptr[13 + c]);
            if (cls_conf > class_conf)
            {
                class_conf = cls_conf;
                class_id = c;
            }
        }
        float conf = obj_conf * class_conf;

        if (conf < conf_threshold)
            continue;

        float x1 = (cx - w / 2 - pad_w) / gain;
        float y1 = (cy - h / 2 - pad_h) / gain;
        float x2 = (cx + w / 2 - pad_w) / gain;
        float y2 = (cy + h / 2 - pad_h) / gain;

        x1 = std::max(0.0f, std::min(x1, static_cast<float>(img_width)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(img_height)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(img_width)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(img_height)));

        detect_box_t box;
        box.x1 = x1;
        box.y1 = y1;
        box.x2 = x2;
        box.y2 = y2;
        box.conf = conf;
        box.cls_id = class_id;

        std::vector<landmark_point_t> landmarks(4);
        // 角点有效性: 模型输出的 4 个角点原始值均需 >= 0 才有效 (与官方一致)。
        // 无效时 (检测模型未输出可信角点) 置 valid=false, 上层退回 bbox 裁剪, 避免透视扭曲。
        bool lm_valid = true;
        for (int j = 0; j < 4 && lm_valid; j++)
        {
            float raw_x = ptr[5 + j * 2];
            float raw_y = ptr[5 + j * 2 + 1];
            if (raw_x < 0.0f || raw_y < 0.0f)
                lm_valid = false;
        }
        for (int j = 0; j < 4; j++)
        {
            float lx = (ptr[5 + j * 2] - pad_w) / gain;
            float ly = (ptr[5 + j * 2 + 1] - pad_h) / gain;
            landmarks[j].x = std::max(0.0f, std::min(lx, static_cast<float>(img_width)));
            landmarks[j].y = std::max(0.0f, std::min(ly, static_cast<float>(img_height)));
            landmarks[j].valid = lm_valid;
        }

        boxes.push_back(box);
        all_landmarks.push_back(landmarks);
        class_ids.push_back(class_id);
        confidences.push_back(conf);
    }

    std::vector<int> keep = nms(boxes, nms_threshold);

    // 车牌类(0/1): 用低阈值 PLATE_NMS_THRESH 再次同类 NMS, 合并重叠的同类别车牌框
    // (全局 nms_threshold 偏大, 同类别重叠车牌往往 IoU 不足而漏合并)
    {
        std::vector<int> plate_boxes_idx;
        for (int i = 0; i < (int)boxes.size(); i++)
            if (boxes[i].cls_id == 0 || boxes[i].cls_id == 1)
                plate_boxes_idx.push_back(i);
        if (!plate_boxes_idx.empty()) {
            std::vector<detect_box_t> plate_boxes;
            for (int i : plate_boxes_idx) plate_boxes.push_back(boxes[i]);
            std::vector<int> plate_keep_local = nms(plate_boxes, plate_nms_threshold);
            std::vector<int> plate_keep;
            for (int li : plate_keep_local) plate_keep.push_back(plate_boxes_idx[li]);
            // 保留非车牌框 + 合并后的车牌框
            std::vector<int> final_keep;
            for (int idx : keep) {
                bool is_plate = (boxes[idx].cls_id == 0 || boxes[idx].cls_id == 1);
                if (!is_plate) final_keep.push_back(idx);
            }
            for (int idx : plate_keep)
                if (std::find(keep.begin(), keep.end(), idx) != keep.end())
                    final_keep.push_back(idx);
            keep = final_keep;
        }
    }

    // 车牌类(0/1)之间跨类别 NMS: 同一物理车牌若同时输出单层/双层框则合并保留一个
    {
        std::vector<int> plate_keep = cross_class_plate_nms(boxes, keep, nms_threshold);
        // 从 keep 中剔除被跨类别 NMS 抑制的车牌框 (保留非车牌框不变)
        std::vector<int> final_keep;
        for (int idx : keep)
        {
            bool is_plate = (boxes[idx].cls_id == 0 || boxes[idx].cls_id == 1);
            bool in_plate_keep = is_plate &&
                std::find(plate_keep.begin(), plate_keep.end(), idx) != plate_keep.end();
            if (!is_plate || in_plate_keep)
                final_keep.push_back(idx);
        }
        keep = final_keep;
    }

    det_results->count = 0;
    for (int idx : keep)
    {
        if (det_results->count >= OBJ_NUMB_MAX_SIZE)
            break;

        detect_result_t &result = det_results->results[det_results->count];
        result.box = boxes[idx];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        for (int j = 0; j < 4; j++)
        {
            result.landmarks[j] = all_landmarks[idx][j];
        }
        det_results->count++;
    }

    delete[] output_data;
    rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

    return 0;
}

cv::Mat letterbox(cv::Mat &img, int new_shape, cv::Scalar color)
{
    int h = img.rows;
    int w = img.cols;
    if (img.empty() || h <= 0 || w <= 0 || new_shape <= 0)
    {
        return cv::Mat();
    }

    float r = static_cast<float>(new_shape) / std::max(h, w);

    int new_h = static_cast<int>(h * r);
    int new_w = static_cast<int>(w * r);
    if (new_h <= 0 || new_w <= 0)
    {
        return cv::Mat();
    }

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h), 0, 0,
               r < 1.0f ? cv::INTER_AREA : cv::INTER_LINEAR);

    int dw = new_shape - new_w;
    int dh = new_shape - new_h;
    int top = dh / 2;
    int bottom = dh - top;
    int left = dw / 2;
    int right = dw - left;

    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, top, bottom, left, right, cv::BORDER_CONSTANT, color);

    return padded;
}

cv::Mat perspective_transform(cv::Mat &img, const std::vector<cv::Point2f> &points)
{
    if (points.size() != 4)
        return img.clone();

    std::vector<cv::Point2f> rect = order_points(points);
    const cv::Point2f &tl = rect[0];
    const cv::Point2f &tr = rect[1];
    const cv::Point2f &br = rect[2];
    const cv::Point2f &bl = rect[3];

    float width_a = std::hypot(br.x - bl.x, br.y - bl.y);
    float width_b = std::hypot(tr.x - tl.x, tr.y - tl.y);
    float height_a = std::hypot(tr.x - br.x, tr.y - br.y);
    float height_b = std::hypot(tl.x - bl.x, tl.y - bl.y);

    int max_width = std::max(1, static_cast<int>(std::round(std::max(width_a, width_b))));
    int max_height = std::max(1, static_cast<int>(std::round(std::max(height_a, height_b))));

    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),
        cv::Point2f(static_cast<float>(max_width - 1), 0),
        cv::Point2f(static_cast<float>(max_width - 1), static_cast<float>(max_height - 1)),
        cv::Point2f(0, static_cast<float>(max_height - 1))
    };

    cv::Mat M = cv::getPerspectiveTransform(rect, dst_points);
    cv::Mat warped;
    cv::warpPerspective(img, warped, M, cv::Size(max_width, max_height));
    return warped;
}

cv::Mat split_double_plate(cv::Mat &plate_img)
{
    if (plate_img.empty())
    {
        return plate_img.clone();
    }

    int h = plate_img.rows;
    int w = plate_img.cols;

    int upper_end = static_cast<int>(5.0 / 12 * h);
    int lower_start = static_cast<int>(1.0 / 3 * h);
    if (upper_end <= 0 || lower_start >= h || upper_end > lower_start)
    {
        return plate_img.clone();
    }

    cv::Mat upper = plate_img(cv::Range(0, upper_end), cv::Range::all());
    cv::Mat lower = plate_img(cv::Range(lower_start, h), cv::Range::all());
    if (upper.empty() || lower.empty() || lower.cols <= 0 || lower.rows <= 0)
    {
        return plate_img.clone();
    }

    cv::Mat upper_resized;
    cv::resize(upper, upper_resized, cv::Size(lower.cols, lower.rows));

    cv::Mat result;
    cv::hconcat(upper_resized, lower, result);
    return result;
}
