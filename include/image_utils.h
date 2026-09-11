#ifndef _RKNN_DEMO_IMAGE_UTILS_H_
#define _RKNN_DEMO_IMAGE_UTILS_H_

#include "common.h"

int get_image_size(image_buffer_t *img);
int read_image(const char *path, image_buffer_t *img);   // stb 读图 -> RGB888
int write_image(const char *path, image_buffer_t *img);  // stb 写 png

// letterbox 缩放: 任意源 -> dst(已分配 virt_addr)
int convert_image_with_letterbox(image_buffer_t *src, image_buffer_t *dst,
                                 letterbox_t *lb, int bg_color);

// NV21(NV12) YUV420SP -> RGB888 (视频帧接入用)
int convert_yuv420sp_to_rgb(image_buffer_t *yuv, image_buffer_t *rgb);

#endif
