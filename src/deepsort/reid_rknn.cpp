#include "reid_rknn.h"
#include "npu_lock.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

ReidRknn::ReidRknn()
    : _ctx(0), _io_num(), _in_attrs(nullptr), _out_attrs(nullptr), _inputs(), _outputs(),
      _in_w(0), _in_h(0), _feat_dim(0), _inited(false) {
}

ReidRknn::~ReidRknn() {
    deinit();
}

int ReidRknn::init(const char *model_path, int feat_dim, int in_w, int in_h) {
    if (_inited) deinit();
    _feat_dim = feat_dim;
    _in_w = in_w;
    _in_h = in_h;

    FILE *fp = fopen(model_path, "rb");
    if (!fp) {
        printf("[REID] fopen %s fail!\n", model_path);
        return -1;
    }
    fseek(fp, 0, SEEK_END);
    long model_len = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    void *model = malloc(model_len);
    if (model_len != (long)fread(model, 1, model_len, fp)) {
        printf("[REID] fread %s fail!\n", model_path);
        fclose(fp);
        free(model);
        return -1;
    }
    fclose(fp);

    int ret = rknn_init(&_ctx, model, model_len, 0, NULL);
    free(model);
    if (ret < 0) {
        printf("[REID] rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    ret = rknn_query(_ctx, RKNN_QUERY_IN_OUT_NUM, &_io_num, sizeof(_io_num));
    if (ret < 0) {
        printf("[REID] rknn_query IO_NUM fail! ret=%d\n", ret);
        rknn_destroy(_ctx);
        return -1;
    }

    _in_attrs = (rknn_tensor_attr *)calloc(_io_num.n_input, sizeof(rknn_tensor_attr));
    _out_attrs = (rknn_tensor_attr *)calloc(_io_num.n_output, sizeof(rknn_tensor_attr));
    for (uint32_t i = 0; i < _io_num.n_input; i++) {
        _in_attrs[i].index = i;
        ret = rknn_query(_ctx, RKNN_QUERY_INPUT_ATTR, &_in_attrs[i], sizeof(rknn_tensor_attr));
        if (ret < 0) {
            printf("[REID] query input attr %u fail ret=%d\n", i, ret);
            rknn_destroy(_ctx);
            return -1;
        }
    }
    for (uint32_t i = 0; i < _io_num.n_output; i++) {
        _out_attrs[i].index = i;
        ret = rknn_query(_ctx, RKNN_QUERY_OUTPUT_ATTR, &_out_attrs[i], sizeof(rknn_tensor_attr));
        if (ret < 0) {
            printf("[REID] query output attr %u fail ret=%d\n", i, ret);
            rknn_destroy(_ctx);
            return -1;
        }
    }

    // 自动从模型输出属性读取特征维数 (输出张量总元素数 = 特征向量长度)
    if (_io_num.n_output > 0 && _out_attrs[0].n_elems > 0)
        _feat_dim = (int)_out_attrs[0].n_elems;
    if (_feat_dim <= 0) {
        printf("[REID] invalid feat_dim=%d (auto from model output failed)\n", _feat_dim);
        rknn_destroy(_ctx);
        return -1;
    }

    memset(_inputs, 0, sizeof(_inputs));
    memset(_outputs, 0, sizeof(_outputs));
    _inputs[0].index = 0;
    _inputs[0].type = RKNN_TENSOR_UINT8;
    _inputs[0].fmt = RKNN_TENSOR_NHWC;
    _inputs[0].size = (uint32_t)(_in_w * _in_h * 3);
    _outputs[0].want_float = 1;   // 直接拿 float 特征

    _inited = true;
    printf("[REID] init OK model=%s feat_dim=%d in=%dx%d\n", model_path, _feat_dim, _in_w, _in_h);
    return 0;
}

void ReidRknn::deinit() {
    if (!_inited) return;
    if (_io_num.n_output > 0) rknn_outputs_release(_ctx, _io_num.n_output, _outputs);
    rknn_destroy(_ctx);
    if (_in_attrs) { free(_in_attrs); _in_attrs = nullptr; }
    if (_out_attrs) { free(_out_attrs); _out_attrs = nullptr; }
    _inited = false;
}

int ReidRknn::infer(const cv::Mat &crop, std::vector<float> &feat) {
    feat.resize(_feat_dim);
    return infer(crop, feat.data(), _feat_dim);
}

int ReidRknn::infer(const cv::Mat &crop, float *feat, int feat_dim) {
    if (!_inited || crop.empty()) return -1;

    cv::Mat rgb;
    if (crop.channels() == 3) {
        // 假设已是 RGB; 若 BGR 需转换 (car_lib 用 BGR, 这里 caller 保证 RGB)
        if (crop.type() == CV_8UC3)
            rgb = crop;
        else
            cv::cvtColor(crop, rgb, cv::COLOR_BGR2RGB);
    } else if (crop.channels() == 1) {
        cv::cvtColor(crop, rgb, cv::COLOR_GRAY2RGB);
    } else {
        return -1;
    }

    cv::Mat resized;
    if (rgb.cols != _in_w || rgb.rows != _in_h)
        cv::resize(rgb, resized, cv::Size(_in_w, _in_h));
    else
        resized = rgb;

    _inputs[0].buf = resized.data;
    // 串行化 NPU 会话: 与 YOLO yolov8_infer 共用 g_npu_lock, 避免并发 rknn_run
    // 导致驱动堆损坏 (跨线程 Eigen free 段错误根因)。
    pthread_mutex_lock(&npu_lock());
    int ret = rknn_inputs_set(_ctx, _io_num.n_input, _inputs);
    if (ret < 0) {
        printf("[REID] rknn_inputs_set fail! ret=%d\n", ret);
        pthread_mutex_unlock(&npu_lock());
        return -1;
    }
    ret = rknn_run(_ctx, nullptr);
    if (ret < 0) {
        printf("[REID] rknn_run fail! ret=%d\n", ret);
        pthread_mutex_unlock(&npu_lock());
        return -1;
    }
    // 每次重新清零输出描述符 (复用成员数组, 避免携带上次 release 后的陈旧 buf/size,
    // 与 car_lib inference_detect_model 每帧 memset 局部 outputs 的用法一致)
    memset(_outputs, 0, sizeof(_outputs));
    _outputs[0].index = 0;
    _outputs[0].want_float = 1;
    ret = rknn_outputs_get(_ctx, _io_num.n_output, _outputs, NULL);
    if (ret < 0) {
        printf("[REID] rknn_outputs_get fail! ret=%d\n", ret);
        pthread_mutex_unlock(&npu_lock());
        return -1;
    }

    int n = feat_dim < _feat_dim ? feat_dim : _feat_dim;
    float *p = (float *)_outputs[0].buf;
    for (int i = 0; i < n; i++) feat[i] = p[i];
    rknn_outputs_release(_ctx, _io_num.n_output, _outputs);
    pthread_mutex_unlock(&npu_lock());
    return 0;
}
