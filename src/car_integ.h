#ifndef _CAR_INTEG_H_
#define _CAR_INTEG_H_

#include "yolov8.h"
#include "car_worker.h"

// 车牌/车辆识别集成模块 (主线程侧):
//   - 初始化解耦后的 car_worker (独立识别线程, 从 chn 拉帧)
//   - car_integ_run 消费 worker 最新识别结果: 坐标映射 -> IOU 关联 -> 计票择优更新
//   - 维护按 track_id 的持久化属性表, 每帧回填, 供 OSD/[PLATE] 稳定显示
//
// 调用方(post_thread)每帧调用 car_integ_run(), 传入最新 YOLO 检测结果;
// 内部非阻塞拉取 worker 结果, 无新结果时仅做每帧回填, 开销极低。

typedef struct {
    int enable;            // 1=启用
    int debug;             // 1=打印识别详细调试信息; 0=仅输出车牌结果
    int plate_type;        // 车牌号码识别引擎: 0=plate_rec(REC), 1=LPRNet
    int src_mode;          // 保留字段(统一走 VPSS 取帧), 不再区分 VI/VPSS
    float roi_nx, roi_ny;  // ROI 左上角 (0-1 归一化, 相对源帧; 用户配置)
    float roi_nw, roi_nh;  // ROI 尺寸 (0-1 归一化; 用户配置)
    int roi_x, roi_y;      // ROI 左上角 (源帧像素, 由归一化换算; 4K: 0..g_vi_w)
    int roi_w, roi_h;      // ROI 尺寸 (源帧像素)
    int chn;               // 车牌 VPSS 通道号 (默认 2)
    int interval_ms;       // 识别间隔 (抽帧), 默认 1000
    int ai_w, ai_h;        // YOLO AI 流分辨率 (用于坐标映射到 AI 坐标系)
    float detect_conf_thresh;  // 车牌检测置信度阈值, 默认 0.3
    float detect_nms_thresh;   // 车牌检测全局 NMS 阈值, 默认 0.5
    float plate_nms_thresh;    // 车牌类专用低阈值 NMS, 默认 0.1 (合并重叠同类别车牌框)
    int  plate_vote_thresh;    // 车牌计票择优阈值, 默认 3 (连续 N 次一致才采信)
    int  car_color_vote_thresh;// 车色计票阈值, 默认 2 (更宽松)
    int  max_track_age_frames; // track 属性条目淘汰帧数, 默认 90 (约 3s@30fps)
    float match_iou_thresh;    // 车牌->车辆 IOU 关联阈值, 默认 0.1
    int  save_plate_image;     // 1=检出车牌时保存检测图片 (整帧+车牌裁剪 JPEG)
    char save_dir[256];        // 保存目录 (默认 /userdata/yolo/captures/)
    int  save_interval_ms;     // 保存限频 (两次保存最小间隔, 默认 5000ms)
    char detect_model[256];
    char plate_model[256];
    char car_model[256];
    char plate_color[16];  // 中文: 黑色/蓝色/绿色/白色/黄色
    char car_color[16];    // 中文: 黑色/蓝色/.../白色
} car_integ_cfg_t;

// 初始化: 启动 car_worker 识别线程 (失败返回非 0)
int  car_integ_init(const car_integ_cfg_t *cfg);
// 停止 worker 并释放资源
void car_integ_deinit(void);
// 消费 worker 最新识别结果: 坐标映射 -> IOU 关联 -> 更新持久化 track 属性
// 返回 0 成功, 非 0 失败(本轮无结果可处理)
int  car_integ_run(object_detect_result_list *od);

// 取最近一次识别出的原始检测结果 (final_result_t, 供 TCP 输出等外部消费)。
// rect 为 ROI 相对坐标, 需加 g_car_cfg.roi_x/y 映射回整幅; fw/fh 为识别帧(整幅)尺寸。
// 返回 1=有新结果, 0=暂无(未初始化或无结果)。
int  car_integ_get_last_raw(std::vector<final_result_t> *results,
                            int *fw, int *fh, long *seq);

// 最近一次车牌识别管线耗时 (ms), 供日志打印 (无识别时返回最近一次值)
double car_integ_get_last_ms(void);

// 当前配置 (供上层打印/调试)
extern car_integ_cfg_t g_car_cfg;

// 本次识别出的车牌检测框 (主码流坐标, 供 OSD 叠加)
typedef struct {
    int x, y, x2, y2;          // 车牌框 (主码流像素坐标)
    char plate_number[32];     // 车牌号
    char plate_color[16];      // 车牌色 (中文, LPRNet 模式为空)
    char car_color[16];        // 车身色 (中文, 可能为空)
    float conf;                // 车牌检测置信度 (0~1)
    int  class_type;           // 0=SinglePlate 1=DoublePlate 2=Car
    int  valid;
} car_plate_box_t;

#define CAR_PLATE_BOX_MAX 16
extern car_plate_box_t g_car_plate_boxes[CAR_PLATE_BOX_MAX];
extern int g_car_plate_box_cnt;

#endif
