#ifndef REID_RKNN_H
#define REID_RKNN_H

#include "rknn_api.h"
#include <opencv2/opencv.hpp>
#include <vector>

// RKNN ReID 特征提取器 (OSNet 等 512 维, 输入 128x256)
// 采用与 car_lib 一致的 rknn_inputs_set/rknn_run/rknn_outputs_get 非零拷贝方式
class ReidRknn {
public:
    ReidRknn();
    ~ReidRknn();

    // model_path: .rknn 路径; feat_dim: 输出特征维数; in_w/in_h: 模型输入尺寸
    int init(const char *model_path, int feat_dim, int in_w, int in_h);
    void deinit();

    // 对单张裁剪图 (任意尺寸) 做 resize + 推理, 输出 feat_dim 维特征
    int infer(const cv::Mat &crop, std::vector<float> &feat);
    int infer(const cv::Mat &crop, float *feat, int feat_dim);

    int width() const { return _in_w; }
    int height() const { return _in_h; }
    int featDim() const { return _feat_dim; }

private:
    rknn_context _ctx;
    rknn_input_output_num _io_num;
    rknn_tensor_attr *_in_attrs;
    rknn_tensor_attr *_out_attrs;
    rknn_input _inputs[1];
    rknn_output _outputs[1];
    int _in_w, _in_h, _feat_dim;
    bool _inited;
};

#endif // REID_RKNN_H
