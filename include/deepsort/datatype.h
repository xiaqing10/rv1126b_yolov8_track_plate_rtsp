#ifndef DEEPSORTDATATYPE_H
#define DEEPSORTDATATYPE_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

// 运行时 ReID 特征维数 (由模型实际输出决定, 不再硬编码 512)
// 在 deepsort_init() 时根据 rknn 输出属性自动设置, 所有特征矩阵使用该动态维数。
extern int g_feat_dim;

#ifndef DETECTBOX_DEFINED
#define DETECTBOX_DEFINED
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor | Eigen::DontAlign> DETECTBOXSS;
#endif
typedef struct CLSCONF {
    CLSCONF() {
        this->cls = -1;
        this->conf = -1;
    }
    CLSCONF(int cls, float conf) {
        this->cls = cls;
        this->conf = conf;
    }
    int cls;
    float conf;
} CLSCONF;

// 特征矩阵使用动态列数 (等于 g_feat_dim)
// 注意: 动态尺寸矩阵必须带 Eigen::DontAlign。
// 32 位 armv7 下 Eigen DenseStorage 析构按 (_Options&DontAlign)==0 决定是否走
// handmade_aligned_free (读 data-1 处基址)。若分配/释放不匹配或 data 前无预留槽,
// free(data-1) 会堆越界(ASan: heap-buffer-overflow)。加 DontAlign 后一律走 plain
// malloc/free, 杜绝该类崩溃。
typedef Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor | Eigen::DontAlign> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Eigen::DontAlign> FEATURESS;
//typedef std::vector<FEATURE> FEATURESS;

//MyKalmanFilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 8, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

//main
using RESULT_DATA = std::pair<int, DETECTBOX>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t{
    std::vector<MATCH_DATA> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;
}TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor | Eigen::DontAlign> DYNAMICM;

#endif //DEEPSORTDATATYPE_H