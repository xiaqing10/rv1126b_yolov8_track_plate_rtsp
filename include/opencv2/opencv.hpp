// Minimal OpenCV compatibility shim for RV1126B (no full OpenCV).
// Provides only the types ByteTrack needs: cv::Rect_<float> and cv::Scalar.
// Also pulls in common STL headers that real opencv.hpp includes transitively.
#ifndef OPENCV_OPENCV_HPP_SHIMMIN
#define OPENCV_OPENCV_HPP_SHIMMIN

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace cv {

template <typename _Tp>
class Rect_ {
public:
    _Tp x, y, width, height;
    Rect_() : x(0), y(0), width(0), height(0) {}
    Rect_(_Tp _x, _Tp _y, _Tp _w, _Tp _h) : x(_x), y(_y), width(_w), height(_h) {}
    template <typename T2>
    operator Rect_<T2>() const { return Rect_<T2>((T2)x, (T2)y, (T2)width, (T2)height); }
};

typedef Rect_<float>  Rect2f;
typedef Rect_<int>    Rect2i;
typedef Rect_<double> Rect2d;
typedef Rect2i Rect;

class Scalar {
public:
    double val[4];
    Scalar() { val[0] = val[1] = val[2] = val[3] = 0; }
    Scalar(double v0, double v1, double v2, double v3 = 0) {
        val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
    }
};

} // namespace cv

#endif
