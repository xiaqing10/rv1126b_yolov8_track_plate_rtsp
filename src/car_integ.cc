// =====================================================================
// 车牌/车辆识别 与 YOLO 主流程集成模块 (主线程侧)
//
//   识别在独立 car_worker 线程中进行 (从 chn 拉全图 -> car_lib 识别)。
//   本模块 (car_integ) 负责主线程侧:
//     - 消费 worker 最新识别结果 (非阻塞轮询)
//     - 坐标映射 (ROI 相对 -> 整幅 -> AI) + IOU 关联到 track_id
//     - 维护按 track_id 的持久化属性表 + 计票择优 (抗误识别)
//     - 每帧回填已采信的车牌/车色到 od, 供 OSD/[PLATE] 稳定显示
//
// 依赖: car_worker.h (识别线程), yolov8.h/postprocess.h (object_detect_result)
// =====================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include "car_integ.h"

// ---------------- 全局配置 ----------------
car_integ_cfg_t g_car_cfg;

static car_worker_t g_car_worker;
static int g_car_inited = 0;
static long g_frame_cnt = 0;   // 全局帧计数 (驱动 track 属性条目淘汰)

// 车牌检测框 (源帧坐标, 供 OSD 叠加; 由 car_integ_run 刷新, rgn_update 读取)
car_plate_box_t g_car_plate_boxes[CAR_PLATE_BOX_MAX];
int g_car_plate_box_cnt = 0;

// 最近一次原始识别结果 (供 TCP 输出等外部消费), 带锁保护
static pthread_mutex_t g_raw_lock = PTHREAD_MUTEX_INITIALIZER;
static std::vector<final_result_t> g_last_raw;
static int g_last_raw_fw = 0, g_last_raw_fh = 0;
static long g_last_raw_seq = 0;

// 最近一次车牌识别管线耗时 (ms), 供日志打印
static double g_car_last_ms = 0.0;
double car_integ_get_last_ms(void) { return g_car_last_ms; }

// ---------------- 按 track_id 的持久化属性表 (计票择优) ----------------
#define MAX_TRACK_ATTR 512
typedef struct {
    int  active;                    // 条目在用
    long last_seen_frame;           // 最近一次识别命中的帧计数
    // 车牌号
    char plate_number[32];
    int  plate_votes;               // 当前候选累计票数 (一致则+, 冲突则-)
    int  plate_adopted;             // 1=已采信 (达到阈值, 写入 od)
    // 车牌色
    char plate_color[16];
    int  plate_color_votes;
    // 车色
    char car_color[16];
    int  car_color_votes;
    int  car_color_adopted;
} track_attr_t;
static track_attr_t g_track_attr[MAX_TRACK_ATTR];

// 英文 -> 中文 颜色名映射 (与 car_recognition 管线输出对应)
static const char *plate_color_cn(const char *en) {
    if (!strcmp(en, "Black")) return "黑色";
    if (!strcmp(en, "Blue"))  return "蓝色";
    if (!strcmp(en, "Green")) return "绿色";
    if (!strcmp(en, "White")) return "白色";
    if (!strcmp(en, "Yellow"))return "黄色";
    return en;
}
static const char *car_color_cn(const char *en) {
    if (!strcmp(en, "Black"))  return "黑色";
    if (!strcmp(en, "Blue"))   return "蓝色";
    if (!strcmp(en, "Yellow")) return "黄色";
    if (!strcmp(en, "Brown"))  return "棕色";
    if (!strcmp(en, "Green"))  return "绿色";
    if (!strcmp(en, "Gray"))   return "灰色";
    if (!strcmp(en, "Orange")) return "橙色";
    if (!strcmp(en, "Pink"))   return "粉色";
    if (!strcmp(en, "Purple")) return "紫色";
    if (!strcmp(en, "Red"))    return "红色";
    if (!strcmp(en, "White"))  return "白色";
    return en;
}

// ---------------- IOU / 距离工具 ----------------
static float iou_box(float ax1, float ay1, float ax2, float ay2,
                     float bx1, float by1, float bx2, float by2)
{
    float x1 = ax1 > bx1 ? ax1 : bx1, y1 = ay1 > by1 ? ay1 : by1;
    float x2 = ax2 < bx2 ? ax2 : bx2, y2 = ay2 < by2 ? ay2 : by2;
    float inter = (x2 > x1 ? x2 - x1 : 0) * (y2 > y1 ? y2 - y1 : 0);
    float uni = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter;
    if (uni <= 0) return 0.0f;
    return inter / uni;
}

// 主YOLO(kezhu-det 10类) 中需要回填车牌/车色的车辆类别 (0基):
// 2=小车 3=厢车 4=巴士 5=卡车 6=摩托; 其余(人/自行车/三轮车等)视为非目标
static inline int is_vehicle_cls(int cls_id)
{
    return cls_id == 2 || cls_id == 3 || cls_id == 4 || cls_id == 5 || cls_id == 6;
}

// ---------------- 对外接口 ----------------
int car_integ_init(const car_integ_cfg_t *cfg)
{
    if (!cfg || !cfg->enable) return 0;

    memset(&g_track_attr, 0, sizeof(g_track_attr));
    g_car_plate_box_cnt = 0;
    g_frame_cnt = 0;

    g_car_cfg = *cfg;
    if (g_car_cfg.interval_ms <= 0) g_car_cfg.interval_ms = 1000;
    if (g_car_cfg.plate_vote_thresh <= 0) g_car_cfg.plate_vote_thresh = 3;
    if (g_car_cfg.car_color_vote_thresh <= 0) g_car_cfg.car_color_vote_thresh = 2;
    if (g_car_cfg.max_track_age_frames <= 0) g_car_cfg.max_track_age_frames = 90;
    if (g_car_cfg.match_iou_thresh <= 0.0f) g_car_cfg.match_iou_thresh = 0.1f;
    if (g_car_cfg.roi_w <= 0 || g_car_cfg.roi_h <= 0) {
        printf("[CAR] invalid roi %dx%d\n", g_car_cfg.roi_w, g_car_cfg.roi_h);
        return -1;
    }

    // 组装 worker 配置
    memset(&g_car_worker, 0, sizeof(g_car_worker));
    g_car_worker.enable = g_car_cfg.enable;
    g_car_worker.chn = g_car_cfg.chn;
    g_car_worker.interval_ms = g_car_cfg.interval_ms;
    g_car_worker.plate_type = g_car_cfg.plate_type;
    g_car_worker.roi_x = g_car_cfg.roi_x;
    g_car_worker.roi_y = g_car_cfg.roi_y;
    g_car_worker.roi_w = g_car_cfg.roi_w;
    g_car_worker.roi_h = g_car_cfg.roi_h;
    g_car_worker.detect_conf_thresh = g_car_cfg.detect_conf_thresh;
    g_car_worker.detect_nms_thresh  = g_car_cfg.detect_nms_thresh;
    g_car_worker.plate_nms_thresh   = g_car_cfg.plate_nms_thresh;
    g_car_worker.save_plate_image   = g_car_cfg.save_plate_image;
    g_car_worker.save_interval_ms   = g_car_cfg.save_interval_ms;
    snprintf(g_car_worker.save_dir, sizeof(g_car_worker.save_dir), "%s", g_car_cfg.save_dir);
    snprintf(g_car_worker.detect_model, sizeof(g_car_worker.detect_model), "%s", g_car_cfg.detect_model);
    snprintf(g_car_worker.plate_model,  sizeof(g_car_worker.plate_model),  "%s", g_car_cfg.plate_model);
    snprintf(g_car_worker.car_model,    sizeof(g_car_worker.car_model),    "%s", g_car_cfg.car_model);

    int ret = car_worker_init(&g_car_worker);
    if (ret != 0) {
        printf("[CAR] car_worker_init fail %d (disable car recognition)\n", ret);
        return -1;
    }
    g_car_inited = 1;
    printf("[CAR] car_integ init OK roi=(%d,%d,%d,%d) chn=%d interval=%dms "
           "plate_type=%s votes=plate:%d color:%d match_iou=%.2f\n",
           g_car_cfg.roi_x, g_car_cfg.roi_y, g_car_cfg.roi_w, g_car_cfg.roi_h,
           g_car_cfg.chn, g_car_cfg.interval_ms,
           g_car_cfg.plate_type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" : "plate_rec",
           g_car_cfg.plate_vote_thresh, g_car_cfg.car_color_vote_thresh,
           g_car_cfg.match_iou_thresh);
    return 0;
}

void car_integ_deinit(void)
{
    if (g_car_inited) {
        car_worker_deinit(&g_car_worker);
        g_car_inited = 0;
    }
}

// 查/取 track 属性条目; 若条目年龄超限则复位(视为新 track, 防 id 复用串数据)
static track_attr_t *attr_for_track(int track_id)
{
    if (track_id < 0 || track_id >= MAX_TRACK_ATTR) return NULL;
    track_attr_t *a = &g_track_attr[track_id];
    if (!a->active || g_frame_cnt - a->last_seen_frame > g_car_cfg.max_track_age_frames) {
        memset(a, 0, sizeof(*a));
        a->active = 1;
    }
    a->last_seen_frame = g_frame_cnt;
    return a;
}

// 把已采信的车牌/车色回填到当帧 od (每帧调用, 保证 OSD/[PLATE] 稳定显示)
static void apply_track_attrs(object_detect_result_list *od)
{
    for (int i = 0; i < od->count; i++) {
        object_detect_result *r = &od->results[i];
        if (!is_vehicle_cls(r->cls_id) || r->track_id < 0) continue;
        track_attr_t *a = attr_for_track(r->track_id);
        if (!a) continue;
        if (a->plate_adopted && a->plate_number[0]) {
            r->has_plate = 1;
            snprintf(r->plate_number, sizeof(r->plate_number), "%s", a->plate_number);
            snprintf(r->plate_color, sizeof(r->plate_color), "%s", a->plate_color);
        }
        if (a->car_color_adopted && a->car_color[0])
            snprintf(r->car_color, sizeof(r->car_color), "%s", a->car_color);
    }
}

// 主流程调用 (每帧调用): 非阻塞拉取 worker 结果 -> 关联 -> 计票择优 -> 回填
int car_integ_run(object_detect_result_list *od)
{
    if (!g_car_inited) return -1;
    g_frame_cnt++;

    // 每帧先回填已采信属性 (即使本轮无新识别结果, 也保持 OSD 稳定)
    if (od) apply_track_attrs(od);

    std::vector<final_result_t> results;
    double elapsed = 0.0;
    int seq = 0;
    int fw = 0, fh = 0;
    if (car_worker_get_result(&g_car_worker, &results, &elapsed, &seq, &fw, &fh) != 1)
        return 0;   // 无新结果, 仅回填

    g_car_last_ms = elapsed;

    // 整幅帧尺寸 (worker 实际拉取的 chn 帧) -> AI 坐标映射
    float s_ai = (g_car_cfg.ai_w > 0 && fw > 0) ? (float)g_car_cfg.ai_w / (float)fw : 0.0f;
    if (s_ai <= 0.0f) return -1;

    // 缓存原始识别结果 (供 TCP 输出消费)
    pthread_mutex_lock(&g_raw_lock);
    g_last_raw = results;
    g_last_raw_fw = fw; g_last_raw_fh = fh; g_last_raw_seq = seq;
    pthread_mutex_unlock(&g_raw_lock);

    g_car_plate_box_cnt = 0;

    // 收集车辆与车牌 (整幅像素坐标; worker 输出为 ROI 相对坐标, 需加 roi 偏移)
    struct {
        float cx, cy, x1, y1, x2, y2;
        std::string car_color;
        std::string plate_number, plate_color;
        int has_plate;
    } cars[OBJ_NUMB_MAX_SIZE]; int ncar = 0;

    struct {
        float x1, y1, x2, y2;      // 车牌框 (整幅像素)
        std::string number, color;
    } plates[OBJ_NUMB_MAX_SIZE]; int nplate = 0;

    for (size_t i = 0; i < results.size(); i++) {
        const final_result_t &r = results[i];
        float fx = g_car_cfg.roi_x + r.rect.x;
        float fy = g_car_cfg.roi_y + r.rect.y;
        float rw = r.rect.width, rh = r.rect.height;
        if (r.class_type == 2) {
            if (ncar >= OBJ_NUMB_MAX_SIZE) continue;
            cars[ncar].x1 = fx; cars[ncar].y1 = fy;
            cars[ncar].x2 = fx + rw; cars[ncar].y2 = fy + rh;
            cars[ncar].cx = fx + rw / 2; cars[ncar].cy = fy + rh / 2;
            cars[ncar].car_color = r.car_color;
            cars[ncar].has_plate = 0;
            ncar++;
        } else {
            if (nplate >= OBJ_NUMB_MAX_SIZE) continue;
            plates[nplate].x1 = fx; plates[nplate].y1 = fy;
            plates[nplate].x2 = fx + rw; plates[nplate].y2 = fy + rh;
            plates[nplate].number = r.plate_number;
            plates[nplate].color = r.plate_color;
            nplate++;
        }
    }

    // 刷新车牌检测框 (整幅坐标, 供 OSD 叠加)
    for (size_t i = 0; i < results.size() && g_car_plate_box_cnt < CAR_PLATE_BOX_MAX; i++) {
        const final_result_t &r = results[i];
        if (r.class_type == 2) continue;
        car_plate_box_t *pb = &g_car_plate_boxes[g_car_plate_box_cnt];
        pb->x = g_car_cfg.roi_x + (int)r.rect.x;
        pb->y = g_car_cfg.roi_y + (int)r.rect.y;
        pb->x2 = pb->x + (int)r.rect.width;
        pb->y2 = pb->y + (int)r.rect.height;
        pb->valid = 1;
        pb->conf = r.confidence;
        pb->class_type = r.class_type;
        snprintf(pb->plate_number, sizeof(pb->plate_number), "%s", r.plate_number.c_str());
        snprintf(pb->plate_color, sizeof(pb->plate_color), "%s",
                 plate_color_cn(r.plate_color.c_str()));
        pb->car_color[0] = 0;
        g_car_plate_box_cnt++;
    }

    // ---- 关联 + 计票择优: 每个识别结果绑定到一个 track_id ----
    // 1) 先处理有车辆检测的: 车框 IOU -> od 车辆 track_id
    // 2) 车牌: 优先绑到同帧最近车辆; 无车辆时退而求其次按车牌框与 od 车辆 IOU 直绑
    for (int p = 0; p < nplate; p++) {
        if (plates[p].number.empty()) continue;

        int track_id = -1;

        // 同帧车辆: 车牌中心最近的车
        int best_car = -1; float best_d = 1e18f;
        for (int c = 0; c < ncar; c++) {
            float pcx = (plates[p].x1 + plates[p].x2) / 2;
            float pcy = (plates[p].y1 + plates[p].y2) / 2;
            float dx = pcx - cars[c].cx, dy = pcy - cars[c].cy;
            float d = dx * dx + dy * dy;
            if (d < best_d) { best_d = d; best_car = c; }
        }
        if (best_car >= 0) {
            // 该车 -> od 车辆 track_id (IOU, 车框缩放回 AI 坐标)
            int bid = -1; float best_iou = g_car_cfg.match_iou_thresh;
            for (int i = 0; i < od->count; i++) {
                const object_detect_result *r = &od->results[i];
                if (!is_vehicle_cls(r->cls_id) || r->track_id < 0) continue;
                float ax1 = cars[best_car].x1 * s_ai, ay1 = cars[best_car].y1 * s_ai;
                float ax2 = cars[best_car].x2 * s_ai, ay2 = cars[best_car].y2 * s_ai;
                float iou = iou_box(ax1, ay1, ax2, ay2,
                                    r->box.left, r->box.top, r->box.right, r->box.bottom);
                if (iou > best_iou) { best_iou = iou; bid = i; }
            }
            if (bid >= 0) track_id = od->results[bid].track_id;
        }

        // 兜底: 无车辆(或车未匹配上)时, 用车牌框直接与 od 车辆 IOU (AI 坐标)
        if (track_id < 0) {
            int bid = -1; float best_iou = g_car_cfg.match_iou_thresh;
            float px1 = plates[p].x1 * s_ai, py1 = plates[p].y1 * s_ai;
            float px2 = plates[p].x2 * s_ai, py2 = plates[p].y2 * s_ai;
            for (int i = 0; i < od->count; i++) {
                const object_detect_result *r = &od->results[i];
                if (!is_vehicle_cls(r->cls_id) || r->track_id < 0) continue;
                float iou = iou_box(px1, py1, px2, py2,
                                    r->box.left, r->box.top, r->box.right, r->box.bottom);
                if (iou > best_iou) { best_iou = iou; bid = i; }
            }
            if (bid >= 0) track_id = od->results[bid].track_id;
        }

        if (track_id < 0) continue;

        // 计票择优: 与已采信/候选车牌比较
        track_attr_t *a = attr_for_track(track_id);
        if (!a) continue;
        const char *new_num = plates[p].number.c_str();
        if (a->plate_number[0] == 0) {
            snprintf(a->plate_number, sizeof(a->plate_number), "%s", new_num);
            a->plate_votes = 1;
        } else if (!strcmp(a->plate_number, new_num)) {
            if (a->plate_votes < 999) a->plate_votes++;
        } else {
            if (a->plate_votes <= 1) {
                snprintf(a->plate_number, sizeof(a->plate_number), "%s", new_num);
                a->plate_votes = 1;
            } else {
                a->plate_votes--;
            }
        }
        if (!a->plate_adopted && a->plate_votes >= g_car_cfg.plate_vote_thresh) {
            a->plate_adopted = 1;
            snprintf(a->plate_color, sizeof(a->plate_color), "%s",
                     plate_color_cn(plates[p].color.c_str()));
            printf("[CAR] track %d 采纳车牌 %s(%s) 票=%d\n",
                   track_id, a->plate_number, a->plate_color, a->plate_votes);
        }
    }

    // 车色: 计票择优
    for (int c = 0; c < ncar; c++) {
        if (cars[c].car_color.empty()) continue;
        int bid = -1; float best_iou = g_car_cfg.match_iou_thresh;
        float ax1 = cars[c].x1 * s_ai, ay1 = cars[c].y1 * s_ai;
        float ax2 = cars[c].x2 * s_ai, ay2 = cars[c].y2 * s_ai;
        for (int i = 0; i < od->count; i++) {
            const object_detect_result *r = &od->results[i];
            if (!is_vehicle_cls(r->cls_id) || r->track_id < 0) continue;
            float iou = iou_box(ax1, ay1, ax2, ay2,
                                r->box.left, r->box.top, r->box.right, r->box.bottom);
            if (iou > best_iou) { best_iou = iou; bid = i; }
        }
        if (bid < 0) continue;
        int track_id = od->results[bid].track_id;
        track_attr_t *a = attr_for_track(track_id);
        if (!a) continue;
        const char *new_col = car_color_cn(cars[c].car_color.c_str());
        if (a->car_color[0] == 0) {
            snprintf(a->car_color, sizeof(a->car_color), "%s", new_col);
            a->car_color_votes = 1;
        } else if (!strcmp(a->car_color, new_col)) {
            if (a->car_color_votes < 999) a->car_color_votes++;
        } else {
            if (a->car_color_votes <= 1) {
                snprintf(a->car_color, sizeof(a->car_color), "%s", new_col);
                a->car_color_votes = 1;
            } else {
                a->car_color_votes--;
            }
        }
        if (!a->car_color_adopted && a->car_color_votes >= g_car_cfg.car_color_vote_thresh) {
            a->car_color_adopted = 1;
        }
    }

    // 本轮识别出的车牌, 若已采信则打印一次 (非 debug)
    if (g_car_cfg.debug)
        printf("[CAR] frame roi cars=%d plates=%d elapsed=%.1fms\n",
               ncar, nplate, elapsed);

    return 0;
}

int car_integ_get_last_raw(std::vector<final_result_t> *results,
                           int *fw, int *fh, long *seq)
{
    if (!g_car_inited) return 0;
    pthread_mutex_lock(&g_raw_lock);
    int have = (!g_last_raw.empty() || g_last_raw_seq > 0) ? 1 : 0;
    if (have) {
        if (results) *results = g_last_raw;
        if (fw) *fw = g_last_raw_fw;
        if (fh) *fh = g_last_raw_fh;
        if (seq) *seq = g_last_raw_seq;
    }
    pthread_mutex_unlock(&g_raw_lock);
    return have;
}
