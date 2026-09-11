// 车牌/车辆识别 worker 线程层实现
//   独立线程从 VPSS chn 拉全图 -> car_lib 识别 -> 结果投递队列
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include "rk_mpi_vpss.h"
#include "rk_mpi_mb.h"

#include "car_worker.h"

// 整幅 NV12(带行距) -> 连续 BGR cv::Mat
static int nv12_to_bgr(const unsigned char *y_src, int w, int h, int stride,
                       cv::Mat &bgr_out)
{
    if (!y_src || w <= 0 || h <= 0) return -1;

    // 约束偶对齐 (NV12 UV 采样要求)
    int ow = w & ~1;
    int oh = h & ~1;
    if (ow <= 0 || oh <= 0) return -1;

    cv::Mat nv12(oh * 3 / 2, ow, CV_8UC1);
    unsigned char *dst = nv12.data;
    const unsigned char *ys = y_src;
    for (int yy = 0; yy < oh; yy++)
        memcpy(dst + (size_t)yy * ow, ys + (size_t)yy * stride, ow);

    const unsigned char *uvs = y_src + (size_t)h * stride;
    unsigned char *dstu = dst + (size_t)oh * ow;
    for (int yy = 0; yy < oh / 2; yy++)
        memcpy(dstu + (size_t)yy * ow, uvs + (size_t)yy * stride, ow);

    cv::cvtColor(nv12, bgr_out, cv::COLOR_YUV2BGR_NV12);
    return 0;
}

// 递归创建目录 (mkdir -p)
static void mkdir_p(const char *path)
{
    if (!path || !*path) return;
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);
    size_t len = strlen(tmp);
    if (len && tmp[len - 1] == '/') tmp[len - 1] = '\0';
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (access(tmp, F_OK) != 0) mkdir(tmp, 0755);
            *p = '/';
        }
    }
    if (access(tmp, F_OK) != 0) mkdir(tmp, 0755);
}

// 保存车牌检测图片: 整帧 + 车牌裁剪 (JPEG)。
// 坐标: final_result_t.rect 为 ROI 相对坐标, 需加 roi_x/roi_y 映射回全图。
static void save_plate_snapshot(car_worker_t *w, const cv::Mat &full_bgr,
                                const std::vector<final_result_t> &results)
{
    if (!w->save_plate_image) return;

    mkdir_p(w->save_dir);
    char ts[64];
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    snprintf(ts, sizeof(ts), "%04d%02d%02d_%02d%02d%02d",
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);

    char base[384];
    snprintf(base, sizeof(base), "%s/%s_%d", w->save_dir, ts, w->seq);

    // 整帧 (带 ROI 检测区域示意)
    cv::Mat full = full_bgr;
    if (w->roi_w > 0 && w->roi_h > 0 && w->roi_x >= 0 && w->roi_y >= 0) {
        cv::Rect roi(w->roi_x, w->roi_y, w->roi_w, w->roi_h);
        cv::rectangle(full, roi, cv::Scalar(0, 0, 255), 2);
    }
    char full_path[400];
    snprintf(full_path, sizeof(full_path), "%s_full.jpg", base);
    if (!cv::imwrite(full_path, full))
        printf("[CAR] save full fail: %s\n", full_path);

    // 每个车牌裁剪
    for (size_t i = 0; i < results.size(); i++) {
        const final_result_t &r = results[i];
        if (r.class_type == 2) continue;             // 车辆框不裁
        if (r.rect.width <= 0 || r.rect.height <= 0) continue;
        int px = r.rect.x + w->roi_x;
        int py = r.rect.y + w->roi_y;
        cv::Rect plate(px, py, r.rect.width, r.rect.height);
        plate &= cv::Rect(0, 0, full_bgr.cols, full_bgr.rows);
        if (plate.width <= 0 || plate.height <= 0) continue;
        cv::Mat crop = full_bgr(plate);
        char plate_path[400];
        snprintf(plate_path, sizeof(plate_path), "%s_plate%zu_%s.jpg", base, i,
                 r.plate_number.empty() ? "noid" : r.plate_number.c_str());
        if (!cv::imwrite(plate_path, crop))
            printf("[CAR] save plate fail: %s\n", plate_path);
    }
}

static void *car_worker_thread(void *arg)
{
    car_worker_t *w = (car_worker_t *)arg;
    struct timespec t_last;
    clock_gettime(CLOCK_MONOTONIC, &t_last);
    struct timespec t_save_last = t_last;

    while (w->running) {
        // 时间门控: interval_ms 一次
        struct timespec tnow;
        clock_gettime(CLOCK_MONOTONIC, &tnow);
        long long elapsed_ms =
            (long long)(tnow.tv_sec - t_last.tv_sec) * 1000 +
            (tnow.tv_nsec - t_last.tv_nsec) / 1000000;
        if (elapsed_ms < w->interval_ms) {
            usleep(20000);
            continue;
        }
        t_last = tnow;

        VIDEO_FRAME_INFO_S frame;
        memset(&frame, 0, sizeof(frame));
        int ret = RK_MPI_VPSS_GetChnFrame(0, w->chn, &frame, 1000);
        if (ret != RK_SUCCESS) {
            if (w->running) usleep(20000);
            continue;
        }

        unsigned char *vaddr = (unsigned char *)RK_MPI_MB_Handle2VirAddr(frame.stVFrame.pMbBlk);
        if (!vaddr) {
            RK_MPI_VPSS_ReleaseChnFrame(0, w->chn, &frame);
            continue;
        }

        int fw = (int)frame.stVFrame.u32Width;
        int fh = (int)frame.stVFrame.u32Height;
        int stride = (int)frame.stVFrame.u32VirWidth;

        cv::Mat full_bgr;
        int cv_ret = nv12_to_bgr(vaddr, fw, fh, stride, full_bgr);
        RK_MPI_VPSS_ReleaseChnFrame(0, w->chn, &frame);

        if (cv_ret != 0 || full_bgr.empty()) continue;

        std::vector<final_result_t> results;
        double elapsed = 0.0;
        ret = inference_car_recognition_pipeline(&w->pipe, full_bgr, &results, &elapsed);

        // 保存车牌检测图片 (检出车牌且满足限频时; 避免每帧刷屏卡顿)
        if (ret == 0 && w->save_plate_image && !results.empty()) {
            bool has_plate = false;
            for (size_t i = 0; i < results.size(); i++)
                if (results[i].class_type != 2) { has_plate = true; break; }
            int save_iv = (w->save_interval_ms > 0) ? w->save_interval_ms : 5000;
            long long save_elapsed =
                (long long)(tnow.tv_sec - t_save_last.tv_sec) * 1000 +
                (tnow.tv_nsec - t_save_last.tv_nsec) / 1000000;
            if (has_plate && save_elapsed >= save_iv) {
                save_plate_snapshot(w, full_bgr, results);
                t_save_last = tnow;
            }
        }

        pthread_mutex_lock(&w->lock);
        if (ret == 0) {
            w->results = results;
            w->elapsed_ms = elapsed;
            w->frame_w = fw;
            w->frame_h = fh;
            w->seq++;
            w->has_result = 1;
            pthread_cond_broadcast(&w->cond);
        }
        pthread_mutex_unlock(&w->lock);
    }

    return NULL;
}

int car_worker_init(car_worker_t *w)
{
    if (!w || !w->enable) return 0;
    if (w->interval_ms <= 0) w->interval_ms = 1000;
    if (w->chn <= 0) w->chn = 2;
    if (w->roi_w <= 0 || w->roi_h <= 0) {
        printf("[CAR] invalid roi %dx%d\n", w->roi_w, w->roi_h);
        return -1;
    }

    pthread_mutex_init(&w->lock, NULL);
    pthread_cond_init(&w->cond, NULL);

    int ret = init_car_recognition_pipeline(&w->pipe,
                                            w->detect_model, w->plate_model, w->car_model,
                                            w->plate_type,
                                            w->roi_x, w->roi_y, w->roi_w, w->roi_h);
    if (ret != 0) {
        printf("[CAR] init_car_recognition_pipeline fail %d\n", ret);
        return -1;
    }
    if (w->detect_conf_thresh > 0) w->pipe.detect_conf_thresh = w->detect_conf_thresh;
    if (w->detect_nms_thresh  > 0) w->pipe.detect_nms_thresh  = w->detect_nms_thresh;
    if (w->plate_nms_thresh   > 0) w->pipe.plate_nms_thresh   = w->plate_nms_thresh;

    // 保存默认值
    if (w->save_interval_ms <= 0) w->save_interval_ms = 5000;
    if (!w->save_dir[0]) snprintf(w->save_dir, sizeof(w->save_dir), "/userdata/yolo/captures/");

    w->running = 1;
    w->seq = 0;
    w->has_result = 0;
    if (pthread_create(&w->tid, NULL, car_worker_thread, w) != 0) {
        printf("[CAR] pthread_create worker fail\n");
        release_car_recognition_pipeline(&w->pipe);
        w->running = 0;
        return -1;
    }
    printf("[CAR] worker thread started chn=%d interval=%dms roi=(%d,%d,%d,%d) engine=%s\n",
           w->chn, w->interval_ms, w->roi_x, w->roi_y, w->roi_w, w->roi_h,
           w->pipe.plate_type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" : "plate_rec");
    return 0;
}

void car_worker_deinit(car_worker_t *w)
{
    if (!w) return;
    w->running = 0;
    if (w->tid) {
        pthread_cond_broadcast(&w->cond);
        pthread_join(w->tid, NULL);
        w->tid = 0;
    }
    release_car_recognition_pipeline(&w->pipe);
    pthread_cond_destroy(&w->cond);
    pthread_mutex_destroy(&w->lock);
}

int car_worker_get_result(car_worker_t *w, std::vector<final_result_t> *results,
                          double *elapsed_ms, int *seq, int *frame_w, int *frame_h)
{
    if (!w || !w->enable) return -1;
    pthread_mutex_lock(&w->lock);
    if (!w->has_result) {
        pthread_mutex_unlock(&w->lock);
        return 0;
    }
    if (results) *results = w->results;
    if (elapsed_ms) *elapsed_ms = w->elapsed_ms;
    if (seq) *seq = w->seq;
    if (frame_w) *frame_w = w->frame_w;
    if (frame_h) *frame_h = w->frame_h;
    w->has_result = 0;   // 结果被消费一次, 等下一轮新识别再投票
    pthread_mutex_unlock(&w->lock);
    return 1;
}
