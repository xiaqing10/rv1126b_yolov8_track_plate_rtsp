#ifndef _TCP_OUTPUT_H_
#define _TCP_OUTPUT_H_

#include <stddef.h>

// TCP + NDJSON 输出模块 (独立, 仅依赖 POSIX socket, 无 SDK/OpenCV 依赖):
//   - 内置 TCP server, 监听 port, 接受多客户端连接
//   - 每个连接收到用 '\n' 分隔的 JSON (NDJSON) 数据行
//   - 调用方组装好 JSON 行后通过 tcp_output_send_line() 广播给所有在线客户端
//   - 慢客户端会被丢弃(内部按 client_buf 容量), 不影响主线程检测
//
// 协议约定 (由调用方在 yolo_rtsp.cc 组装):
//   frame      每帧跟踪结果, 全类别都发, "cls" 用数字(cls_id), "box" 为 0-1 归一化
//   detection  纯检测(车牌识别 raw), "class" 用数字, "box" 为 0-1 归一化
//   plate_adopted  车牌采信事件(可选)

typedef struct {
    int enable;          // 1=启用 TCP 输出
    int port;            // 监听端口, 默认 9000
    int max_clients;     // 最大并发客户端数, 默认 4
    int client_buf;      // 单客户端发送缓冲(字节), 默认 65536, 写满即断开该客户端
} tcp_output_cfg_t;

// 初始化: 启动监听线程 (返回 0 成功)
int  tcp_output_init(const tcp_output_cfg_t *cfg);
// 停止并释放
void tcp_output_deinit(void);
// 广播一行 JSON (自动追加 '\n'), 无客户端时无操作
void tcp_output_send_line(const char *json_line);

// JSON 字符串转义辅助 (把 str 中的 " \ \n \r \t 转义, 写入 dst, 返回写入长度)
// dst 至少为 len*2+1 字节; 供调用方拼接车牌号等文本字段用
size_t tcp_output_json_escape(const char *str, char *dst, size_t dst_size);

#endif // _TCP_OUTPUT_H_
