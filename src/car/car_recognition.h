#ifndef _RKNN_CAR_RECOGNITION_H_
#define _RKNN_CAR_RECOGNITION_H_

#include "rknn_api.h"
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#define DETECT_MODEL_INPUT_SIZE 384
#define PLATE_MODEL_HEIGHT 48
#define PLATE_MODEL_WIDTH 168
#define CAR_MODEL_SIZE 64

#define LPRNET_MODEL_HEIGHT 24
#define LPRNET_MODEL_WIDTH 94
#define LPRNET_OUT_ROWS 68
#define LPRNET_OUT_COLS 18

#define OBJ_NUMB_MAX_SIZE 128
#define NMS_THRESH 0.5f
#define PLATE_NMS_THRESH 0.1f   // 车牌检测框专用低阈值 NMS, 合并重叠的同类别车牌框
#define BOX_THRESH 0.3f

#define PLATE_CHARS_NUM 78
#define PLATE_SEQ_LEN 21
#define PLATE_COLOR_NUM 5
#define CAR_COLOR_NUM 11

// 车牌号码识别引擎选择
#define CAR_PLATE_TYPE_REC    0   // 使用 car_recognition 的 plate_rec (号码+颜色)
#define CAR_PLATE_TYPE_LPRNET 1   // 使用 LPRNet (仅号码, 颜色置空)

#if defined(RV1106_1103)
#include "dma_alloc.hpp"
typedef struct {
    char *dma_buf_virt_addr;
    int dma_buf_fd;
    int size;
} rknn_dma_buf;
#endif

#if !defined(RKNN_APP_CONTEXT_DEFINED)
#define RKNN_APP_CONTEXT_DEFINED
typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
#if defined(RV1106_1103)
    rknn_tensor_mem *input_mems[1];
    rknn_tensor_mem *output_mems[1];
    rknn_dma_buf img_dma_buf;
#endif
    int model_channel;
    int model_width;
    int model_height;
    bool is_quant;
} rknn_app_context_t;
#endif

typedef struct {
    float x1, y1, x2, y2;
    float conf;
    int cls_id;
} detect_box_t;

typedef struct {
    float x, y;
    bool valid;
} landmark_point_t;

typedef struct {
    detect_box_t box;
    landmark_point_t landmarks[4];
    int class_id;
    float confidence;
} detect_result_t;

typedef struct {
    int count;
    detect_result_t results[OBJ_NUMB_MAX_SIZE];
} detect_result_list_t;

typedef struct {
    std::string plate_number;
    std::string plate_color;
    int color_id;
} plate_result_t;

typedef struct {
    std::string color_name;
    int color_id;
    float confidence;
} car_color_result_t;

typedef struct {
    int class_type;
    cv::Rect rect;
    std::vector<cv::Point2f> landmarks;
    bool landmarks_valid;
    std::string plate_number;
    std::string plate_color;
    std::string car_color;
    float confidence;
    float color_confidence;
} final_result_t;

typedef struct {
    rknn_app_context_t detect_ctx;
    rknn_app_context_t plate_ctx;    // plate_rec (号码+颜色), plate_type==REC 时使用
    rknn_app_context_t lprnet_ctx;   // LPRNet (仅号码), plate_type==LPRNET 时使用
    rknn_app_context_t car_ctx;
    int plate_type;                  // CAR_PLATE_TYPE_REC / CAR_PLATE_TYPE_LPRNET
    float detect_conf_thresh;        // 检测置信度阈值 (默认 BOX_THRESH)
    float detect_nms_thresh;         // 检测全局 NMS 阈值 (默认 NMS_THRESH)
    float plate_nms_thresh;          // 车牌类专用低阈值 NMS (默认 PLATE_NMS_THRESH)
    // ROI (全图坐标, 相对传入 inference 的整幅图)。仅在 roi_w/roi_h 均 >0 时启用,
    // 库内在该区域内检测/识别, 输出坐标为 ROI 相对坐标 (调用方需加 roi_x/y 映射回全图)。
    int roi_x, roi_y, roi_w, roi_h;
    bool initialized;
} car_recognition_pipeline_t;

extern const std::vector<std::string> plate_chars;
extern const std::vector<std::string> plate_colors;
extern const std::vector<std::string> car_colors;
extern const std::vector<std::string> class_names;
extern const std::vector<std::string> lprnet_plate_code;

int init_detect_model(const char *model_path, rknn_app_context_t *app_ctx);
int release_detect_model(rknn_app_context_t *app_ctx);
int inference_detect_model(rknn_app_context_t *app_ctx, cv::Mat &img, detect_result_list_t *det_results, 
                           float conf_threshold, float nms_threshold, float plate_nms_threshold);

int init_plate_rec_model(const char *model_path, rknn_app_context_t *app_ctx);
int release_plate_rec_model(rknn_app_context_t *app_ctx);
int inference_plate_rec_model(rknn_app_context_t *app_ctx, cv::Mat &plate_img, plate_result_t *plate_result);

int init_lprnet_model(const char *model_path, rknn_app_context_t *app_ctx);
int release_lprnet_model(rknn_app_context_t *app_ctx);
int inference_lprnet_model(rknn_app_context_t *app_ctx, cv::Mat &plate_img, std::string *plate_number);

int init_car_color_model(const char *model_path, rknn_app_context_t *app_ctx);
int release_car_color_model(rknn_app_context_t *app_ctx);
int inference_car_color_model(rknn_app_context_t *app_ctx, cv::Mat &car_img, car_color_result_t *color_result);

cv::Mat letterbox(cv::Mat &img, int new_shape, cv::Scalar color);
cv::Mat perspective_transform(cv::Mat &img, const std::vector<cv::Point2f> &points);
cv::Mat split_double_plate(cv::Mat &plate_img);

int init_car_recognition_pipeline(car_recognition_pipeline_t *pipeline,
                                  const char *detect_model_path,
                                  const char *plate_model_path,
                                  const char *car_model_path,
                                  int plate_type,
                                  int roi_x, int roi_y, int roi_w, int roi_h);
int release_car_recognition_pipeline(car_recognition_pipeline_t *pipeline);
int inference_car_recognition_pipeline(car_recognition_pipeline_t *pipeline,
                                       cv::Mat &img,
                                       std::vector<final_result_t> *final_results,
                                       double *elapsed_ms);

#endif
