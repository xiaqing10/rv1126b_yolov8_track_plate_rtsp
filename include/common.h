#ifndef _RKNN_DEMO_COMMON_H_
#define _RKNN_DEMO_COMMON_H_

#include <stdint.h>
#include "rknn_api.h"

typedef enum {
    IMAGE_FORMAT_RGB888 = 0,
    IMAGE_FORMAT_BGR888,
    IMAGE_FORMAT_GRAY8,
    IMAGE_FORMAT_RGBA8888,
    IMAGE_FORMAT_BGRA8888,
    IMAGE_FORMAT_YUV420SP_NV21,
    IMAGE_FORMAT_YUV420SP_NV12,
    IMAGE_FORMAT_UNKNOWN
} image_format_t;

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} image_rect_t;

typedef struct {
    image_format_t format;
    int width;
    int height;
    int width_stride;
    int height_stride;
    int size;
    int fd;
    unsigned char *virt_addr;
} image_buffer_t;

typedef struct {
    float scale;
    float x_pad;
    float y_pad;
} letterbox_t;

const char *get_type_string(rknn_tensor_type type);
const char *get_format_string(rknn_tensor_format fmt);
const char *get_qnt_type_string(rknn_tensor_qnt_type qnt);

#endif
