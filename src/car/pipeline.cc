#include <algorithm>
#include <chrono>
#include <string.h>
#include <vector>

#include "car_recognition.h"

// 探测车牌识别模型类型, 据此自动选择引擎。
// 判定依据(多信号结合, 保证两套模型都能用):
//   1) 输出张量总数能被 78 整除   -> REC  (78类, seq*78)
//   2) 输出个数 >= 2              -> REC  (REC 才有颜色第二路输出; LPRNet 只有1路)
//   3) 输出张量总数能被 68 整除   -> LPRNet (68类, 68*18=1224)
// 返回: CAR_PLATE_TYPE_REC / CAR_PLATE_TYPE_LPRNET / -1(无法判定)
static int probe_plate_model_type(const char *model_path)
{
    int model_len = 0;
    char *model = NULL;
    rknn_context ctx = 0;
    int type = -1;

    FILE *fp = fopen(model_path, "rb");
    if (fp == NULL) return -1;
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz <= 0) { fclose(fp); return -1; }
    model = (char *)malloc(sz);
    if (model == NULL) { fclose(fp); return -1; }
    if (fread(model, 1, sz, fp) != (size_t)sz) { free(model); fclose(fp); return -1; }
    fclose(fp);
    model_len = (int)sz;

    if (rknn_init(&ctx, model, model_len, 0, NULL) != RKNN_SUCC) { free(model); return -1; }
    free(model);

    rknn_input_output_num io_num;
    if (rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) == RKNN_SUCC &&
        io_num.n_output > 0)
    {
        rknn_tensor_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.index = 0;
        if (rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)) == RKNN_SUCC &&
            attr.n_elems > 0)
        {
            int total = (int)attr.n_elems;
            int rec_classes = (int)plate_chars.size();        // 78
            int lpr_classes = (int)lprnet_plate_code.size();  // 68

            if (total % rec_classes == 0 || (int)io_num.n_output >= 2)
                type = CAR_PLATE_TYPE_REC;
            else if (total % lpr_classes == 0)
                type = CAR_PLATE_TYPE_LPRNET;

            printf("[CAR] probe: model outputs=%d out0_elems=%d -> %s\n",
                   (int)io_num.n_output, total,
                   type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" :
                   type == CAR_PLATE_TYPE_REC ? "plate_rec" : "unknown");
        }
    }

    rknn_destroy(ctx);
    return type;
}

int init_car_recognition_pipeline(car_recognition_pipeline_t *pipeline,
                                  const char *detect_model_path,
                                  const char *plate_model_path,
                                  const char *car_model_path,
                                  int plate_type,
                                  int roi_x, int roi_y, int roi_w, int roi_h)
{
    if (pipeline == NULL)
    {
        return -1;
    }

    memset(pipeline, 0, sizeof(*pipeline));
    pipeline->plate_type = plate_type;
    pipeline->detect_conf_thresh = BOX_THRESH;
    pipeline->detect_nms_thresh  = NMS_THRESH;
    pipeline->plate_nms_thresh   = PLATE_NMS_THRESH;
    pipeline->roi_x = roi_x;
    pipeline->roi_y = roi_y;
    pipeline->roi_w = roi_w;
    pipeline->roi_h = roi_h;

    // 自动校正引擎类型: 以所加载模型实际输出为准, 避免 config 与模型不匹配导致解码失败。
    // 保证 REC(78类, 21*78=1638) 与 LPRNet(68类, 68*18=1224) 两个模型都能用。
    int probe_type = probe_plate_model_type(plate_model_path);
    if (probe_type >= 0)
    {
        if (probe_type != pipeline->plate_type)
        {
            printf("[CAR] 自动切换车牌引擎: %s -> %s (按模型输出判定)\n",
                   pipeline->plate_type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" : "plate_rec",
                   probe_type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" : "plate_rec");
            pipeline->plate_type = probe_type;
        }
    }
    else
    {
        printf("[CAR] warn: 无法判定车牌模型类型, 按 config 引擎继续 (%s)\n",
               pipeline->plate_type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" : "plate_rec");
    }

    int ret = init_detect_model(detect_model_path, &pipeline->detect_ctx);
    if (ret != 0)
    {
        return ret;
    }

    if (pipeline->plate_type == CAR_PLATE_TYPE_LPRNET)
    {
        ret = init_lprnet_model(plate_model_path, &pipeline->lprnet_ctx);
        if (ret != 0)
        {
            release_detect_model(&pipeline->detect_ctx);
            memset(&pipeline->detect_ctx, 0, sizeof(pipeline->detect_ctx));
            return ret;
        }
    }
    else
    {
        ret = init_plate_rec_model(plate_model_path, &pipeline->plate_ctx);
        if (ret != 0)
        {
            release_detect_model(&pipeline->detect_ctx);
            memset(&pipeline->detect_ctx, 0, sizeof(pipeline->detect_ctx));
            return ret;
        }
    }

    ret = init_car_color_model(car_model_path, &pipeline->car_ctx);
    if (ret != 0)
    {
        release_detect_model(&pipeline->detect_ctx);
        if (pipeline->plate_type == CAR_PLATE_TYPE_LPRNET)
        {
            release_lprnet_model(&pipeline->lprnet_ctx);
            memset(&pipeline->lprnet_ctx, 0, sizeof(pipeline->lprnet_ctx));
        }
        else
        {
            release_plate_rec_model(&pipeline->plate_ctx);
            memset(&pipeline->plate_ctx, 0, sizeof(pipeline->plate_ctx));
        }
        memset(&pipeline->detect_ctx, 0, sizeof(pipeline->detect_ctx));
        return ret;
    }

    pipeline->initialized = true;
    return 0;
}

int release_car_recognition_pipeline(car_recognition_pipeline_t *pipeline)
{
    if (pipeline == NULL)
    {
        return -1;
    }

    release_detect_model(&pipeline->detect_ctx);
    if (pipeline->plate_type == CAR_PLATE_TYPE_LPRNET)
    {
        release_lprnet_model(&pipeline->lprnet_ctx);
    }
    else
    {
        release_plate_rec_model(&pipeline->plate_ctx);
    }
    release_car_color_model(&pipeline->car_ctx);
    memset(pipeline, 0, sizeof(*pipeline));
    return 0;
}

int inference_car_recognition_pipeline(car_recognition_pipeline_t *pipeline,
                                       cv::Mat &img,
                                       std::vector<final_result_t> *final_results,
                                       double *elapsed_ms)
{
    if (pipeline == NULL || final_results == NULL || !pipeline->initialized || img.empty())
    {
        return -1;
    }

    auto start = std::chrono::high_resolution_clock::now();

    // 库内 ROI 裁剪: 仅在 roi_w/roi_h 均 >0 时启用。裁剪后统一在 roi_img 上检测,
    // 输出坐标为 ROI 相对坐标 (调用方需加 roi_x/y 映射回全图)。
    cv::Mat roi_img = img;
    if (pipeline->roi_w > 0 && pipeline->roi_h > 0)
    {
        cv::Rect roi_rect(pipeline->roi_x, pipeline->roi_y,
                          pipeline->roi_w, pipeline->roi_h);
        roi_rect &= cv::Rect(0, 0, img.cols, img.rows);
        if (roi_rect.width <= 0 || roi_rect.height <= 0)
        {
            return -1;
        }
        roi_img = img(roi_rect);
    }

    detect_result_list_t det_results;
    int ret = inference_detect_model(&pipeline->detect_ctx, roi_img, &det_results,
                                     pipeline->detect_conf_thresh, pipeline->detect_nms_thresh,
                                     pipeline->plate_nms_thresh);
    if (ret != 0)
    {
        return ret;
    }

    final_results->clear();
    final_results->reserve(det_results.count);

    for (int i = 0; i < det_results.count; ++i)
    {
        detect_result_t *det = &det_results.results[i];
        final_result_t final_res;
        final_res.class_type = det->class_id;
        final_res.rect = cv::Rect(det->box.x1, det->box.y1,
                                  det->box.x2 - det->box.x1,
                                  det->box.y2 - det->box.y1);
        final_res.confidence = det->confidence;
        final_res.color_confidence = 0.0f;
        final_res.landmarks_valid = det->landmarks[0].valid;

        for (int j = 0; j < 4; ++j)
        {
            final_res.landmarks.push_back(cv::Point2f(det->landmarks[j].x, det->landmarks[j].y));
        }

        if (det->class_id == 2)
        {
            cv::Rect roi = final_res.rect & cv::Rect(0, 0, roi_img.cols, roi_img.rows);
            if (roi.area() > 0)
            {
                cv::Mat car_img = roi_img(roi);
                if (!car_img.empty())
                {
                    car_color_result_t color_res = {};
                    ret = inference_car_color_model(&pipeline->car_ctx, car_img, &color_res);
                    if (ret == 0)
                    {
                        final_res.car_color = color_res.color_name;
                        final_res.color_confidence = color_res.confidence;
                    }
                }
            }
        }
        else
        {
            cv::Rect roi = final_res.rect & cv::Rect(0, 0, roi_img.cols, roi_img.rows);
            if (roi.area() > 0 && final_res.landmarks.size() == 4)
            {
                printf("[CAR-PLATE] box=(%d,%d %dx%d) lm=",
                       final_res.rect.x, final_res.rect.y,
                       final_res.rect.width, final_res.rect.height);
                for (const auto &p : final_res.landmarks)
                    printf("(%.0f,%.0f) ", p.x, p.y);
                printf("valid=%d\n", final_res.landmarks_valid ? 1 : 0);
                // 角点无效时退回 bbox 四角, 只做轴对齐裁剪, 避免无效角点造成透视扭曲。
                std::vector<cv::Point2f> warp_pts = final_res.landmarks;
                if (!final_res.landmarks_valid)
                {
                    float x1 = final_res.rect.x, y1 = final_res.rect.y;
                    float x2 = final_res.rect.x + final_res.rect.width;
                    float y2 = final_res.rect.y + final_res.rect.height;
                    warp_pts = { cv::Point2f(x1, y1), cv::Point2f(x2, y1),
                                 cv::Point2f(x2, y2), cv::Point2f(x1, y2) };
                }
                cv::Mat warped = perspective_transform(roi_img, warp_pts);
                if (!warped.empty())
                {
                    if (det->class_id == 1)
                    {
                        cv::Mat split_result = split_double_plate(warped);
                        if (!split_result.empty())
                        {
                            warped = split_result;
                        }
                    }

                    if (pipeline->plate_type == CAR_PLATE_TYPE_LPRNET)
                    {
                        std::string plate_number;
                        ret = inference_lprnet_model(&pipeline->lprnet_ctx, warped, &plate_number);
                        if (ret == 0)
                        {
                            final_res.plate_number = plate_number;
                            final_res.plate_color.clear();
                        }
                    }
                    else
                    {
                        plate_result_t plate_res;
                        ret = inference_plate_rec_model(&pipeline->plate_ctx, warped, &plate_res);
                        if (ret == 0)
                        {
                            final_res.plate_number = plate_res.plate_number;
                            final_res.plate_color = plate_res.plate_color;
                        }
                    }
                }
            }
        }

        final_results->push_back(final_res);
    }

    auto end = std::chrono::high_resolution_clock::now();
    if (elapsed_ms != NULL)
    {
        *elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    }

    return 0;
}
