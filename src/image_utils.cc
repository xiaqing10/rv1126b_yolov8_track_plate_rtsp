#include "image_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"

#define CLIP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

int get_image_size(image_buffer_t *img)
{
    if (!img) return 0;
    int ch = 3;
    switch (img->format) {
    case IMAGE_FORMAT_RGB888:
    case IMAGE_FORMAT_BGR888: ch = 3; break;
    case IMAGE_FORMAT_RGBA8888:
    case IMAGE_FORMAT_BGRA8888: ch = 4; break;
    case IMAGE_FORMAT_GRAY8: ch = 1; break;
    case IMAGE_FORMAT_YUV420SP_NV21:
    case IMAGE_FORMAT_YUV420SP_NV12: ch = 1; img->size = img->width * img->height * 3 / 2; return img->size;
    default: ch = 3; break;
    }
    int w = img->width_stride ? img->width_stride : img->width;
    img->size = w * img->height * ch;
    return img->size;
}

// 读图并统一转为 RGB888
int read_image(const char *path, image_buffer_t *img)
{
    int w, h, c;
    unsigned char *data = stbi_load(path, &w, &h, &c, 3); // 强制3通道
    if (!data) {
        printf("stbi_load %s fail\n", path);
        return -1;
    }
    img->format = IMAGE_FORMAT_RGB888;
    img->width = w;
    img->height = h;
    img->width_stride = w;
    img->virt_addr = data;
    img->size = w * h * 3;
    img->fd = -1;
    return 0;
}

int write_image(const char *path, image_buffer_t *img)
{
    if (img->format != IMAGE_FORMAT_RGB888) {
        printf("write_image only support RGB888\n");
        return -1;
    }
    int ok = stbi_write_png(path, img->width, img->height, 3,
                            img->virt_addr, img->width * 3);
    return ok ? 0 : -1;
}

static inline unsigned char sat(int v) { return (unsigned char)CLIP(v, 0, 255); }

// letterbox 缩放(最近邻, 静态测试足够; 视频建议后续换RGA)
int convert_image_with_letterbox(image_buffer_t *src, image_buffer_t *dst,
                                 letterbox_t *lb, int bg_color)
{
    if (!src || !dst || !dst->virt_addr) return -1;
    float scale_x = (float)dst->width / src->width;
    float scale_y = (float)dst->height / src->height;
    float scale = scale_x < scale_y ? scale_x : scale_y;

    int new_w = (int)(src->width * scale + 0.5f);
    int new_h = (int)(src->height * scale + 0.5f);
    float pad_w = (dst->width - new_w) / 2.0f;
    float pad_h = (dst->height - new_h) / 2.0f;

    if (lb) { lb->scale = scale; lb->x_pad = pad_w; lb->y_pad = pad_h; }

    // 背景填充
    for (int y = 0; y < dst->height; y++) {
        unsigned char *drow = dst->virt_addr + y * dst->width * 3;
        for (int x = 0; x < dst->width; x++) {
            drow[x * 3 + 0] = bg_color;
            drow[x * 3 + 1] = bg_color;
            drow[x * 3 + 2] = bg_color;
        }
    }

    for (int dy = 0; dy < new_h; dy++) {
        int sy = (int)(dy / scale);
        if (sy >= src->height) sy = src->height - 1;
        unsigned char *srow = src->virt_addr + sy * src->width * 3;
        unsigned char *drow = dst->virt_addr + ((int)(dy + pad_h) * dst->width + (int)pad_w) * 3;
        for (int dx = 0; dx < new_w; dx++) {
            int sx = (int)(dx / scale);
            if (sx >= src->width) sx = src->width - 1;
            drow[dx * 3 + 0] = srow[sx * 3 + 0];
            drow[dx * 3 + 1] = srow[sx * 3 + 1];
            drow[dx * 3 + 2] = srow[sx * 3 + 2];
        }
    }
    return 0;
}

// YUV420SP (NV21/NV12) -> RGB888
int convert_yuv420sp_to_rgb(image_buffer_t *yuv, image_buffer_t *rgb)
{
    int w = yuv->width, h = yuv->height;
    if (!rgb->virt_addr) return -1;
    int stride = yuv->width_stride ? yuv->width_stride : w;
    int uv_off = stride * h;
    unsigned char *y_plane = yuv->virt_addr;
    unsigned char *uv_plane = yuv->virt_addr + uv_off;
    int is_nv21 = (yuv->format == IMAGE_FORMAT_YUV420SP_NV21);

    for (int j = 0; j < h; j++) {
        unsigned char *out = rgb->virt_addr + j * w * 3;
        for (int i = 0; i < w; i++) {
            int Y = y_plane[j * stride + i];
            int uv_i = (j / 2) * stride + (i & ~1);
            int U, V;
            if (is_nv21) { V = uv_plane[uv_i];     U = uv_plane[uv_i + 1]; }
            else        { U = uv_plane[uv_i];     V = uv_plane[uv_i + 1]; }
            int c = Y - 16;
            int d = U - 128;
            int e = V - 128;
            int R = (298 * c + 409 * e + 128) >> 8;
            int G = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int B = (298 * c + 516 * d + 128) >> 8;
            out[i * 3 + 0] = sat(R);
            out[i * 3 + 1] = sat(G);
            out[i * 3 + 2] = sat(B);
        }
    }
    return 0;
}
