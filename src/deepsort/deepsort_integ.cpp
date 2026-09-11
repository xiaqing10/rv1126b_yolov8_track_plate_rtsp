// DeepSORT 异步集成: 后台 ReID worker + 主线程 DeepSORT 匹配
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <queue>
#include <vector>

#include <opencv2/opencv.hpp>

#include "yolov8.h"   // 定义 rknn_app_context_t (postprocess.h 依赖), 必须在 deepsort_integ.h 之前
#include "deepsort_integ.h"
#include "tracker.h"
#include "model.hpp"
#include "reid_rknn.h"
// ---------------- 后台 ReID worker ----------------
// 主线程提交"裁剪图批次"(已 resize 到模型输入), worker 逐张推理, 完成后投递回结果队列。
struct reid_task {
    int seq;
    std::vector<cv::Mat> crops;            // 每张已 resize 到模型输入 (RGB888)
    std::vector<std::vector<float>> feats; // worker 填: 每张 feat_dim 维特征
};

static pthread_mutex_t g_reid_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_reid_cond = PTHREAD_COND_INITIALIZER;
static std::queue<reid_task *> g_task_q;   // 待推理 (main -> worker)
static std::queue<reid_task *> g_done_q;   // 已完成 (worker -> main)
static ReidRknn *g_reid = nullptr;
static pthread_t g_reid_tid;
static int g_reid_running = 0;
static int g_reid_seq = 0;
static int g_reid_interval = 3;   // 每隔 N 帧才对全部目标做一次 ReID 推理
static int g_reid_frame_cnt = 0;  // 帧计数器, 用于节流 ReID 提交

static void *reid_worker_thread(void *arg)
{
    (void)arg;
    printf("[DEEPSORT] ReID worker thread start\n");
    while (g_reid_running) {
        pthread_mutex_lock(&g_reid_lock);
        while (g_task_q.empty() && g_reid_running)
            pthread_cond_wait(&g_reid_cond, &g_reid_lock);
        if (!g_reid_running) { pthread_mutex_unlock(&g_reid_lock); break; }
        reid_task *t = g_task_q.front();
        g_task_q.pop();
        pthread_mutex_unlock(&g_reid_lock);

        t->feats.resize(t->crops.size());
        for (size_t i = 0; i < t->crops.size(); i++)
            g_reid->infer(t->crops[i], t->feats[i]);

        pthread_mutex_lock(&g_reid_lock);
        g_done_q.push(t);
        pthread_cond_signal(&g_reid_cond);
        pthread_mutex_unlock(&g_reid_lock);
    }
    printf("[DEEPSORT] ReID worker thread exit\n");
    return NULL;
}

// 提交一个裁剪批次, 返回本次 seq
static int reid_submit(std::vector<cv::Mat> &crops)
{
    reid_task *t = new reid_task();
    t->seq = ++g_reid_seq;
    t->crops.swap(crops);
    pthread_mutex_lock(&g_reid_lock);
    g_task_q.push(t);
    pthread_cond_signal(&g_reid_cond);
    pthread_mutex_unlock(&g_reid_lock);
    return t->seq;
}

// 非阻塞取回一个已完成批次 (FIFO, 对应上一个提交帧)。无则返回 NULL。
static reid_task *reid_try_get_done(void)
{
    pthread_mutex_lock(&g_reid_lock);
    reid_task *t = nullptr;
    if (!g_done_q.empty()) { t = g_done_q.front(); g_done_q.pop(); }
    pthread_mutex_unlock(&g_reid_lock);
    return t;
}

// ---------------- 全局跟踪器 (主线程独占) ----------------
static tracker *g_tracker = nullptr;
static DETECTIONS g_prev_dets;   // 上一帧检测 (特征由异步回填, 供当前帧匹配)

// 检测框(AI 坐标) -> full_rgb(chn1) 像素坐标
static void ai_to_chn(int ax, int ay, const deepsort_map_t *m, int *cx, int *cy)
{
    float x = (float)(ax - (int)m->roi_ai_x0) / (m->roi_ai_sx > 1e-6 ? m->roi_ai_sx : 1.0f);
    float y = (float)(ay - (int)m->roi_ai_y0) / (m->roi_ai_sy > 1e-6 ? m->roi_ai_sy : 1.0f);
    *cx = (int)x;
    *cy = (int)y;
}

// 裁剪检测框区域 (full_rgb) 并 resize 到模型输入; 含 ReID 方形裁剪微调
static cv::Mat crop_det(const cv::Mat &full, int x1, int y1, int x2, int y2,
                        int in_w, int in_h)
{
    int cx1 = x1, cy1 = y1, cx2 = x2, cy2 = y2;
    int w = cx2 - cx1, h = cy2 - cy1;
    if (w <= 0 || h <= 0) return cv::Mat();

    // ReID 常用方形裁剪: 以框高为基准, 扩宽到 0.5*高 (参考实现)
    int sw = (int)(h * 0.5f);
    if (w < sw) {
        int pad = (sw - w) / 2;
        cx1 -= pad;
        cx2 += (sw - w - pad);
    }
    cv::Rect rc(cx1, cy1, cx2 - cx1, cy2 - cy1);
    rc &= cv::Rect(0, 0, full.cols, full.rows);
    if (rc.width <= 0 || rc.height <= 0) return cv::Mat();
    cv::Mat crop = full(rc).clone();
    if (crop.empty()) return cv::Mat();
    if (crop.cols != in_w || crop.rows != in_h)
        cv::resize(crop, crop, cv::Size(in_w, in_h));
    return crop;
}

// ---------------- 对外 API ----------------
int deepsort_init(const char *reid_model, int feat_dim, int in_w, int in_h, int reid_interval)
{
    if (!reid_model || !*reid_model) return -1;
    g_reid_interval = reid_interval > 0 ? reid_interval : 3;
    g_reid_frame_cnt = 0;
    g_reid = new ReidRknn();
    if (g_reid->init(reid_model, feat_dim, in_w, in_h) != 0) {
        delete g_reid;
        g_reid = nullptr;
        return -1;
    }
    g_feat_dim = g_reid->featDim();   // 以模型实际输出维数为准 (自动读取)
    g_tracker = new tracker(0.2f, 100, 0.7f, 30, 2);  // maxCosineDist, budget, iou, maxAge, nInit
    g_reid_seq = 0;
    while (!g_task_q.empty()) { delete g_task_q.front(); g_task_q.pop(); }
    while (!g_done_q.empty()) { delete g_done_q.front(); g_done_q.pop(); }
    g_reid_running = 1;
    pthread_create(&g_reid_tid, NULL, reid_worker_thread, NULL);
    printf("[DEEPSORT] init OK model=%s feat=%d in=%dx%d\n", reid_model, feat_dim, in_w, in_h);
    return 0;
}

void deepsort_deinit(void)
{
    if (!g_reid_running) return;
    g_reid_running = 0;
    pthread_mutex_lock(&g_reid_lock);
    pthread_cond_broadcast(&g_reid_cond);
    pthread_mutex_unlock(&g_reid_lock);
    pthread_join(g_reid_tid, NULL);
    while (!g_task_q.empty()) { delete g_task_q.front(); g_task_q.pop(); }
    while (!g_done_q.empty()) { delete g_done_q.front(); g_done_q.pop(); }
    delete g_reid;
    g_reid = nullptr;
    delete g_tracker;
    g_tracker = nullptr;
}

int deepsort_update(image_buffer_t *full_rgb, object_detect_result_list *od,
                    const deepsort_map_t *m)
{
    if (!g_reid || !g_tracker || !full_rgb || !od || !m) return -1;
    if (!full_rgb->virt_addr || full_rgb->width <= 0 || full_rgb->height <= 0) return -1;

    cv::Mat full(full_rgb->height, full_rgb->width, CV_8UC3, full_rgb->virt_addr);

    // 1) 当前帧检测 -> DETECTIONS + 裁剪批次
    DETECTIONS det;
    std::vector<cv::Mat> crops;
    det.reserve(od->count);
    crops.reserve(od->count);
    for (int i = 0; i < od->count; i++) {
        const object_detect_result *r = &od->results[i];
        int cx1, cy1, cx2, cy2;
        ai_to_chn(r->box.left, r->box.top, m, &cx1, &cy1);
        ai_to_chn(r->box.right, r->box.bottom, m, &cx2, &cy2);
        cv::Mat c = crop_det(full, cx1, cy1, cx2, cy2, g_reid->width(), g_reid->height());
        if (c.empty()) continue;
        DETECTION_ROW d;
        // 特征默认置为 g_feat_dim 维零行: 动态 FEATURE(1xDynamic) 默认构造为 1x0 空矩阵,
        // 若回填前就传入 tracker, 在 features.row=feature 处会触发 Eigen resize-on-Block 断言。
        d.feature = FEATURE::Zero(1, g_feat_dim);
        d.tlwh(0) = (float)r->box.left;
        d.tlwh(1) = (float)r->box.top;
        d.tlwh(2) = (float)(r->box.right - r->box.left);
        d.tlwh(3) = (float)(r->box.bottom - r->box.top);
        d.confidence = r->prop;
        det.push_back(d);
        crops.push_back(c);
    }

    // 2) 提交当前帧裁剪给 worker (异步, 节流: 每隔 reid_interval 帧才对全部目标推理一次)
    g_reid_frame_cnt++;
    if (!crops.empty() && (g_reid_frame_cnt % g_reid_interval) == 0) reid_submit(crops);

    // 3) 用上一帧特征做匹配 (特征滞后一帧)
    reid_task *done = reid_try_get_done();
    if (done) {
        // 回填上一帧特征
        DETECTIONS &pd = g_prev_dets;
        size_t n = done->crops.size() < pd.size() ? done->crops.size() : pd.size();
        for (size_t i = 0; i < n; i++)
            if (!done->feats[i].empty() && (int)done->feats[i].size() >= g_feat_dim)
                pd[i].feature = Eigen::Map<FEATURE>(done->feats[i].data(), 1, g_feat_dim);

        g_tracker->predict();
        g_tracker->update(pd);

        // 写回当前 od 的 track_id: 用确认轨迹框 IoU 匹配当前检测框 (同一 AI 坐标)
        for (int i = 0; i < od->count; i++) od->results[i].track_id = -1;
        for (Track &tr : g_tracker->tracks) {
            if (!tr.is_confirmed() || tr.time_since_update > 1) continue;
            DETECTBOX tb = tr.to_tlwh();
            float tx = tb(0), ty = tb(1), tw = tb(2), th = tb(3);
            int best = -1; float best_iou = 0;
            for (int i = 0; i < od->count; i++) {
                const object_detect_result *r = &od->results[i];
                float ix = (float)r->box.left, iy = (float)r->box.top;
                float iw = (float)(r->box.right - r->box.left);
                float ih = (float)(r->box.bottom - r->box.top);
                float x1 = tx > ix ? tx : ix, y1 = ty > iy ? ty : iy;
                float x2 = (tx + tw) < (ix + iw) ? (tx + tw) : (ix + iw);
                float y2 = (ty + th) < (iy + ih) ? (ty + th) : (iy + ih);
                float inter = (x2 - x1 > 0 ? x2 - x1 : 0) * (y2 - y1 > 0 ? y2 - y1 : 0);
                float uni = tw * th + iw * ih - inter;
                if (uni <= 0) continue;
                float iou = inter / uni;
                if (iou > best_iou) { best_iou = iou; best = i; }
            }
            if (best >= 0 && best_iou > 0.1f)
                od->results[best].track_id = tr.track_id;
        }
        delete done;
    }

    // 4) 保存当前帧为"上一帧" (供下一帧匹配)
    g_prev_dets.swap(det);

    return (int)g_tracker->tracks.size();
}
