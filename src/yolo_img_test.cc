// 静态图 YOLOv8 推理工具 (验证模型在 RV1126B NPU 上可跑, 也用于调试视觉检测模型)
// 支持可选主线程 ROI(0-1 归一化): 从原图裁剪 ROI -> 缩放到模型尺寸 -> 检测 -> 反偏移回全图坐标,
// 与主程序 yolo_rtsp 的 VPSS 硬件"裁剪+缩放"管线行为一致, 便于离线调试 ROI 检测效果。
// 用法: yolo_img_test <model.rknn> <image> [labels.txt] [out.png] [iters]
//                     [roi_x roi_y roi_w roi_h]   (roi 0-1, 省略或全 0 = 全图检测)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "yolov8.h"
#include "image_utils.h"
#include "file_utils.h"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

// 从源图(src, RGB888 连续)裁剪 roi 区域到新缓冲 (RGB888), 返回 dst; 失败返回 -1
static int crop_rgb(const image_buffer_t *src, int rx, int ry, int rw, int rh,
                    image_buffer_t *dst) {
    if (rx < 0 || ry < 0 || rw <= 0 || rh <= 0 ||
        rx + rw > src->width || ry + rh > src->height) {
        printf("crop: roi(%d,%d %dx%d) 超出源图 %dx%d\n", rx, ry, rw, rh, src->width, src->height);
        return -1;
    }
    memset(dst, 0, sizeof(*dst));
    dst->format = IMAGE_FORMAT_RGB888;
    dst->width = rw;
    dst->height = rh;
    dst->size = rw * rh * 3;
    dst->virt_addr = (unsigned char *)malloc(dst->size);
    if (!dst->virt_addr) { printf("crop: malloc fail\n"); return -1; }
    for (int y = 0; y < rh; y++) {
        memcpy(dst->virt_addr + (size_t)y * rw * 3,
               src->virt_addr + (size_t)(ry + y) * src->width * 3 + rx * 3,
               (size_t)rw * 3);
    }
    return 0;
}

static void draw_box_rgb(image_buffer_t *img, int x1, int y1, int x2, int y2,
                         int r, int g, int b) {
    for (int x = x1; x < x2 && x < img->width; x++)
        for (int t = 0; t < 3 && y1 + t < img->height; t++) {
            unsigned char *p = img->virt_addr + ((y1+t)*img->width + x)*3;
            p[0]=r; p[1]=g; p[2]=b;
            if (y2-t >= 0){unsigned char *q=img->virt_addr+((y2-t)*img->width + x)*3;q[0]=r;q[1]=g;q[2]=b;}
        }
    for (int y = y1; y < y2 && y < img->height; y++)
        for (int t = 0; t < 3; t++) {
            if (x1+t < img->width){unsigned char*p=img->virt_addr+(y*img->width+x1+t)*3;p[0]=r;p[1]=g;p[2]=b;}
            if (x2-t < img->width){unsigned char*p=img->virt_addr+(y*img->width+x2-t)*3;p[0]=r;p[1]=g;p[2]=b;}
        }
}

// 虚线矩形 (ROI 标识, RGB)
static void draw_box_dashed_rgb(image_buffer_t *img, int x, int y, int rw, int rh,
                                int r, int g, int b, int dash, int gap) {
    if (dash <= 0) dash = 8;
    if (gap <= 0) gap = 4;
    int x0 = x, y0 = y, x1 = x + rw - 1, y1 = y + rh - 1;
    for (int xx = x0; xx <= x1; ) {
        int e = xx + dash - 1 < x1 ? xx + dash - 1 : x1;
        for (int px = xx; px <= e; px++) {
            if (y0 >= 0 && y0 < img->height && px >= 0 && px < img->width) {
                unsigned char *p = img->virt_addr + (y0*img->width + px)*3; p[0]=r; p[1]=g; p[2]=b;
            }
            if (y1 >= 0 && y1 < img->height && px >= 0 && px < img->width) {
                unsigned char *p = img->virt_addr + (y1*img->width + px)*3; p[0]=r; p[1]=g; p[2]=b;
            }
        }
        xx = e + 1 + gap;
    }
    for (int yy = y0; yy <= y1; ) {
        int e = yy + dash - 1 < y1 ? yy + dash - 1 : y1;
        for (int py = yy; py <= e; py++) {
            if (py >= 0 && py < img->height && x0 >= 0 && x0 < img->width) {
                unsigned char *p = img->virt_addr + (py*img->width + x0)*3; p[0]=r; p[1]=g; p[2]=b;
            }
            if (py >= 0 && py < img->height && x1 >= 0 && x1 < img->width) {
                unsigned char *p = img->virt_addr + (py*img->width + x1)*3; p[0]=r; p[1]=g; p[2]=b;
            }
        }
        yy = e + 1 + gap;
    }
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("Usage: %s <model.rknn> <image> [labels.txt] [out.png] [iters]\n", argv[0]);
        printf("              [roi_x roi_y roi_w roi_h]  (0-1 归一化, 全0/省略 = 全图检测)\n");
        return -1;
    }
    const char *model_path = argv[1];
    const char *image_path = argv[2];
    const char *label_path = argc > 3 ? argv[3] : NULL;
    const char *out_path   = argc > 4 ? argv[4] : "out.png";
    int iters = argc > 5 ? atoi(argv[5]) : 1;
    float roi_nx = argc > 6 ? (float)atof(argv[6]) : 0.0f;
    float roi_ny = argc > 7 ? (float)atof(argv[7]) : 0.0f;
    float roi_nw = argc > 8 ? (float)atof(argv[8]) : 0.0f;
    float roi_nh = argc > 9 ? (float)atof(argv[9]) : 0.0f;

    rknn_app_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));

    init_post_process(label_path);

    int ret = init_yolov8_model(model_path, &ctx);
    if (ret != 0) {
        printf("init_yolov8_model fail! ret=%d\n", ret);
        return -1;
    }

    image_buffer_t src;
    memset(&src, 0, sizeof(src));
    ret = read_image(image_path, &src);
    if (ret != 0) {
        printf("read_image fail! %s\n", image_path);
        return -1;
    }
    printf("src image: %dx%d\n", src.width, src.height);

    // 主线程 ROI: 从原图裁剪 ROI -> 缩放检测 -> 反偏移回全图 (默认全图)
    int roi_enable = (roi_nw > 0.0f && roi_nh > 0.0f);
    int rx = 0, ry = 0, rw = src.width, rh = src.height;
    image_buffer_t roi_img;
    memset(&roi_img, 0, sizeof(roi_img));
    if (roi_enable) {
        rx = (int)(roi_nx * src.width);
        ry = (int)(roi_ny * src.height);
        rw = (int)(roi_nw * src.width);
        rh = (int)(roi_nh * src.height);
        if (crop_rgb(&src, rx, ry, rw, rh, &roi_img) != 0) {
            printf("ROI 裁剪失败, 退出\n");
            release_yolov8_model(&ctx);
            deinit_post_process();
            free(src.virt_addr);
            return -1;
        }
        printf("ROI: norm=(%.3f,%.3f %.3fx%.3f) -> px=(%d,%d %dx%d), 在ROI上检测\n",
               roi_nx, roi_ny, roi_nw, roi_nh, rx, ry, rw, rh);
    }

    object_detect_result_list od;
    double sum = 0.0;
    for (int it = 0; it < iters; it++) {
        memset(&od, 0, sizeof(od));
        image_buffer_t *in = roi_enable ? &roi_img : &src;
        double t0 = now_ms();
        ret = inference_yolov8_model(&ctx, in, &od);
        double t1 = now_ms();
        if (ret != 0) {
            printf("inference fail! ret=%d\n", ret);
            return -1;
        }
        sum += (t1 - t0);
        if (it == iters - 1) {
            printf("==== %d objects, infer %.1f ms ====\n", od.count, t1 - t0);
            for (int i = 0; i < od.count; i++) {
                object_detect_result *r = &od.results[i];
                printf("  [%d] %-12s %.3f  box=(%d,%d,%d,%d)\n", r->cls_id,
                       cls_to_name(r->cls_id), r->prop,
                       r->box.left, r->box.top, r->box.right, r->box.bottom);
            }
        }
    }
    if (iters > 1) printf("==== avg infer %.1f ms over %d iters ====\n", sum / iters, iters);

    // 反偏移 ROI 检测结果回全图坐标 (仅在 ROI 检测时)
    if (roi_enable) {
        for (int i = 0; i < od.count; i++) {
            object_detect_result *r = &od.results[i];
            r->box.left   += rx;
            r->box.right  += rx;
            r->box.top    += ry;
            r->box.bottom += ry;
        }
        printf("ROI 检测结果已反偏移回全图坐标\n");
    }

    // 画 ROI 虚线框 (绿色) + 检测框 (蓝色)
    if (roi_enable)
        draw_box_dashed_rgb(&src, rx, ry, rw, rh, 0, 255, 0, 8, 4);
    for (int i = 0; i < od.count; i++) {
        object_detect_result *r = &od.results[i];
        draw_box_rgb(&src, r->box.left, r->box.top, r->box.right, r->box.bottom, 0, 0, 255);
    }
    write_image(out_path, &src);
    printf("saved %s\n", out_path);

    if (roi_enable) free(roi_img.virt_addr);
    release_yolov8_model(&ctx);
    deinit_post_process();
    free(src.virt_addr);
    return 0;
}
