#ifndef DEEPSORT_INTEG_H
#define DEEPSORT_INTEG_H

#include "image_utils.h"
#include "yolov8.h"   // 定义 rknn_app_context_t (postprocess.h 依赖), 须在 postprocess.h 之前
#include "postprocess.h"

#ifdef __cplusplus
extern "C" {
#endif

// DeepSORT 异步集成 (ReID 特征滞后一帧):
//   主线程 post_thread 调用 deepsort_update(), 传入当前帧整幅 RGB (chn1 输出) 与检测结果。
//   主线程裁剪每个检测框 -> 提交给后台 ReID worker 推理 (不阻塞主线程 NPU 推理) -> 用上一帧特征做级联匹配。
// 与 byteTrack_update() 并列, 通过配置 tracker=bytetrack|deepsort 切换。

// 坐标映射参数: 检测框为"整幅-AI 坐标"(0..ai_w x 0..ai_h), full_rgb 为 chn1 输出 (g_chn_w x g_chn_h)。
// 裁剪时需做逆反偏移: chn_x = (ai_x - roi_ai_x0) / roi_ai_sx
typedef struct {
    int   ai_w, ai_h;              // 整幅-AI 坐标系基准
    float roi_ai_x0, roi_ai_y0;    // ROI 左上角 (AI 坐标)
    float roi_ai_sx, roi_ai_sy;    // ROI 缩放系数
    int   full_w, full_h;          // full_rgb 实际宽高 (chn1)
} deepsort_map_t;

// 初始化: 加载 ReID 模型并启动 worker 线程。失败返回非 0。
// reid_interval: 每隔 N 帧才对全部目标做一次 ReID 推理 (>=1)。中间帧复用已匹配特征, 省 NPU 提帧率。
int  deepsort_init(const char *reid_model, int feat_dim, int in_w, int in_h, int reid_interval);

// 每帧调用: 裁剪+提交 ReID, 用上一帧特征做 DeepSORT 匹配, 把 track_id 写回 od。
// full_rgb 必须与 od 属同一帧 (chn1 RGB888)。返回当前确认跟踪目标数 (或 <0 出错)。
int  deepsort_update(image_buffer_t *full_rgb, object_detect_result_list *od,
                     const deepsort_map_t *map);

// 停止 worker 线程并释放模型
void deepsort_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DEEPSORT_INTEG_H
