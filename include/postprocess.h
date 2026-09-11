#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_MAX 256          // 支持的最大类别数, 实际类别数运行时从模型读取
#define DEFAULT_LABEL_PATH "/userdata/labels/coco_80_labels_list.txt"

// class rknn_app_context_t;

typedef struct {
    image_rect_t box;
    float prop;
    int cls_id;
    int track_id;   // ByteTrack 跟踪ID, -1=未跟踪
    // ---- 车牌/车辆识别集成 (car_integ) ----
    int  has_plate;          // 1=已绑定车牌
    char plate_number[32];   // 车牌号 (可能为空)
    char plate_color[16];    // 车牌色 (中文)
    char car_color[16];      // 车身色 (中文)
} object_detect_result;

typedef struct {
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

// ---- 运行时可配置参数(由配置文件/初始化设置) ----
extern char   g_label_path[256];   // 标签文件路径
extern int    g_obj_class_num;     // 实际类别数(0=由模型输出自动推断)
extern float  g_box_conf_thresh;   // 置信度阈值
extern float  g_nms_thresh;        // NMS 阈值

int  init_post_process(const char *label_path);
void deinit_post_process();
char *cls_to_name(int cls_id);
int post_process(rknn_app_context_t *app_ctx, void *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);

void deinitPostProcess();
#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
