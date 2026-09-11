// 车牌/车辆识别 单图测试工具 (只链 car_lib, 不依赖 VPSS)
// 检测结果画框后保存为图片, 便于调试视觉模型。
// 用法: car_img_test <detect.rknn> <plate_model.rknn> <car_color.rknn> <image> [plate_type]
//        [roi_x roi_y roi_w roi_h] [out.jpg]   (roi 为 0-1 归一化; 省略或全 0 = 全图检测)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <opencv2/opencv.hpp>

#include "car_recognition.h"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

int main(int argc, char **argv)
{
    if (argc < 5) {
        printf("Usage: %s <detect.rknn> <plate_model.rknn> <car_color.rknn> <image> "
               "[plate_type] [roi_x roi_y roi_w roi_h] [out.jpg]\n", argv[0]);
        printf("  plate_type: 0=plate_rec(REC, 默认), 1=LPRNet\n");
        printf("  roi_x/y/w/h: 0-1 归一化坐标(相对整幅), 全 0 或省略 = 全图检测\n");
        printf("  out.jpg: 结果保存路径(默认 out_car.jpg)\n");
        return -1;
    }
    const char *detect_path = argv[1];
    const char *plate_path  = argv[2];
    const char *car_path    = argv[3];
    const char *image_path  = argv[4];
    int plate_type = argc > 5 ? atoi(argv[5]) : CAR_PLATE_TYPE_REC;
    // ROI 为 0-1 归一化坐标(相对整幅); 省略或全 0 = 全图检测
    float roi_nx = argc > 6 ? (float)atof(argv[6]) : 0.0f;
    float roi_ny = argc > 7 ? (float)atof(argv[7]) : 0.0f;
    float roi_nw = argc > 8 ? (float)atof(argv[8]) : 0.0f;
    float roi_nh = argc > 9 ? (float)atof(argv[9]) : 0.0f;
    const char *out_path = argc > 10 ? argv[10] : "out_car.jpg";

    // 先读图, 用图片实际尺寸把 0-1 ROI 换算为像素 (供 init_car_recognition_pipeline)
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        printf("imread fail! %s\n", image_path);
        return -1;
    }
    printf("image: %dx%d\n", img.cols, img.rows);
    int roi_x = (int)(roi_nx * img.cols);
    int roi_y = (int)(roi_ny * img.rows);
    int roi_w = (int)(roi_nw * img.cols);
    int roi_h = (int)(roi_nh * img.rows);

    car_recognition_pipeline_t pipe;
    memset(&pipe, 0, sizeof(pipe));
    int ret = init_car_recognition_pipeline(&pipe, detect_path, plate_path, car_path,
                                            plate_type, roi_x, roi_y, roi_w, roi_h);
    if (ret != 0) {
        printf("init_car_recognition_pipeline fail! ret=%d\n", ret);
        return -1;
    }
    printf("engine=%s roi=norm(%.3f,%.3f %.3fx%.3f)->px(%d,%d,%d,%d)\n",
           pipe.plate_type == CAR_PLATE_TYPE_LPRNET ? "LPRNet" : "plate_rec",
           roi_nx, roi_ny, roi_nw, roi_nh, roi_x, roi_y, roi_w, roi_h);
    printf("car_img_test v1.0.0 build " __DATE__ " " __TIME__ "\n");

    std::vector<final_result_t> results;
    double elapsed = 0.0;
    double t0 = now_ms();
    ret = inference_car_recognition_pipeline(&pipe, img, &results, &elapsed);
    double t1 = now_ms();
    if (ret != 0) {
        printf("inference fail! ret=%d\n", ret);
        release_car_recognition_pipeline(&pipe);
        return -1;
    }

    printf("==== %d objects, infer %.1f ms (wall %.1f ms) ====\n",
           (int)results.size(), elapsed, t1 - t0);
    for (size_t i = 0; i < results.size(); i++) {
        const final_result_t &r = results[i];
        const char *type = r.class_type == 2 ? "Car" :
                           (r.class_type == 1 ? "DoublePlate" : "SinglePlate");
        printf("  [%zu] %-12s conf=%.3f box=(%d,%d %dx%d)",
               i, type, r.confidence, r.rect.x, r.rect.y, r.rect.width, r.rect.height);
        if (!r.plate_number.empty())
            printf("  plate=%s(%s)", r.plate_number.c_str(), r.plate_color.c_str());
        if (!r.car_color.empty())
            printf("  car_color=%s", r.car_color.c_str());
        printf("\n");
    }

    // 画框 + 保存结果图 (在 img 副本上绘制, 便于调试视觉模型)
    cv::Mat out = img.clone();
    // ROI 虚线框 (绿色)
    if (roi_w > 0 && roi_h > 0) {
        for (int t = 0; t < 8; t++) {  // dash 段长
            int x1 = roi_x + t;
            if (x1 >= 0 && x1 < out.cols) {
                cv::line(out, cv::Point(x1, roi_y), cv::Point(x1, roi_y + roi_h - 1),
                         cv::Scalar(0, 255, 0), 2);
            }
        }
        cv::rectangle(out, cv::Rect(roi_x, roi_y, roi_w, roi_h), cv::Scalar(0, 255, 0), 2);
    }
    // 检测框 + 标签
    for (size_t i = 0; i < results.size(); i++) {
        const final_result_t &r = results[i];
        cv::Scalar color = r.class_type == 2 ? cv::Scalar(255, 0, 0) :   // Car 蓝
                           (r.class_type == 1 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 255)); // 双层红/单层黄
        cv::rectangle(out, r.rect, color, 2);
        std::string label;
        if (!r.plate_number.empty()) {
            label = r.plate_number;
            if (!r.plate_color.empty()) label += "(" + r.plate_color + ")";
        } else if (!r.car_color.empty()) {
            label = "car:" + r.car_color;
        }
        if (!label.empty()) {
            cv::putText(out, label, cv::Point(r.rect.x, r.rect.y > 10 ? r.rect.y - 5 : r.rect.y + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }
    }
    if (cv::imwrite(out_path, out)) {
        printf("saved %s\n", out_path);
    } else {
        printf("save %s fail!\n", out_path);
    }

    release_car_recognition_pipeline(&pipe);
    return 0;
}
