#ifndef _CAR_WORKER_H_
#define _CAR_WORKER_H_

#include <pthread.h>
#include <vector>

#include "car_recognition.h"

// 车牌/车辆识别 worker 线程层 (app 侧):
//   - 独立线程, 周期性地从 VPSS chn 拉取整幅帧 (全图高分辨率)
//   - 交给 car_lib (init/inference_car_recognition_pipeline) 识别, ROI 由库内裁剪
//   - 识别结果放入结果队列, 供主线程拉取 (car_integ 消费并裁决)
// car_worker 不负责 track 关联/更新, 只负责"拉帧 + 识别 + 投递结果"。

typedef struct {
    // ---- 配置 (init 时写入) ----
    int enable;
    int chn;                 // VPSS 通道号 (默认 2)
    int interval_ms;         // 识别周期 (抽帧间隔), 默认 1000
    // ---- car_lib 模型路径与引擎 ----
    int plate_type;
    int roi_x, roi_y, roi_w, roi_h;
    char detect_model[256];
    char plate_model[256];
    char car_model[256];
    float detect_conf_thresh;
    float detect_nms_thresh;
    float plate_nms_thresh;
    // ---- 车牌检测图片保存 ----
    int  save_plate_image;     // 1=检出车牌时保存检测图片 (整帧+车牌裁剪 JPEG)
    char save_dir[256];        // 保存目录 (自动 mkdir -p)
    int  save_interval_ms;     // 保存限频 (两次保存最小间隔, 默认 5000ms)
    // ---- 运行状态 ----
    int running;
    pthread_t tid;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    car_recognition_pipeline_t pipe;   // car_lib 管线 (识别线程独占)
    // ---- 结果队列 (最新一份, 主线程取走) ----
    int has_result;                    // 1=有最新结果待取
    int seq;                           // 结果序号 (递增, 便于主线程判断是否新)
    int frame_w, frame_h;              // 本次识别所用整幅帧尺寸 (坐标映射用)
    std::vector<final_result_t> results;  // ROI 相对坐标 + 车牌号/色/车色
    double elapsed_ms;                 // 本次识别耗时
} car_worker_t;

// 初始化 worker: 加载 car_lib 模型并启动识别线程。失败返回非 0。
int  car_worker_init(car_worker_t *w);
// 停止识别线程并释放模型
void car_worker_deinit(car_worker_t *w);
// 主线程拉取最新识别结果 (非阻塞)。有结果且较上次新则返回 1 并写出,
// 无新结果返回 0, 出错返回 -1。
int  car_worker_get_result(car_worker_t *w, std::vector<final_result_t> *results,
                           double *elapsed_ms, int *seq, int *frame_w, int *frame_h);

#endif // _CAR_WORKER_H_
