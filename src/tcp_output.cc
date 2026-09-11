// TCP + NDJSON 输出模块实现 (独立, 仅依赖 POSIX socket)
//   见 include/tcp_output.h 的协议说明。
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>

#include "tcp_output.h"

#define TCP_OUT_MAX_CLIENTS 8

typedef struct {
    int fd;                 // 客户端 socket, -1=空闲槽
    char *buf;              // 待发送缓冲
    size_t buf_size;        // 缓冲容量
    size_t buf_len;         // 当前已缓冲字节数
} tcp_client_t;

typedef struct {
    int enable;
    int port;
    int max_clients;
    int client_buf;

    int listen_fd;
    pthread_t accept_tid;
    pthread_mutex_t lock;   // 保护 client[] / 全局状态

    tcp_client_t clients[TCP_OUT_MAX_CLIENTS];
} tcp_output_t;

static tcp_output_t g_tcp;

static void set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// 尝试把缓冲全部写往客户端; 写完返回 0, 否则返回 -1 (可能部分写入)
static int flush_client(tcp_client_t *c)
{
    while (c->buf_len > 0) {
        ssize_t n = send(c->fd, c->buf, c->buf_len, 0);
        if (n > 0) {
            memmove(c->buf, c->buf + n, c->buf_len - n);
            c->buf_len -= n;
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return -1;   // 对端慢, 稍后(下次广播)再写
        } else {
            return -1;   // 连接出错/关闭
        }
    }
    return 0;
}

// 关闭并回收客户端槽
static void close_client_locked(int idx)
{
    tcp_client_t *c = &g_tcp.clients[idx];
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
    if (c->buf) {
        free(c->buf);
        c->buf = NULL;
    }
    c->buf_len = 0;
    c->buf_size = 0;
}

static void broadcast_locked(const char *data, size_t len)
{
    for (int i = 0; i < TCP_OUT_MAX_CLIENTS; i++) {
        tcp_client_t *c = &g_tcp.clients[i];
        if (c->fd < 0) continue;

        // 先尝试清空已有积压(对端恢复可写时)
        if (c->buf_len > 0) flush_client(c);

        // 追加新数据
        if (c->buf_len + len > c->buf_size) {
            // 缓冲不足: 慢客户端, 直接断开
            close_client_locked(i);
            continue;
        }
        memcpy(c->buf + c->buf_len, data, len);
        c->buf_len += len;

        // 尝试直接写出(非阻塞)
        flush_client(c);
    }
}

static void *accept_thread(void *arg)
{
    (void)arg;
    while (g_tcp.enable) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(g_tcp.listen_fd, &rfds);
        struct timeval tv = { 1, 0 };   // 每秒醒来检查 enable
        int sel = select(g_tcp.listen_fd + 1, &rfds, NULL, NULL, &tv);
        if (sel <= 0) continue;
        if (!FD_ISSET(g_tcp.listen_fd, &rfds)) continue;

        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        int fd = accept(g_tcp.listen_fd, (struct sockaddr *)&cli, &cli_len);
        if (fd < 0) continue;

        set_nonblock(fd);
        pthread_mutex_lock(&g_tcp.lock);

        // 找空闲槽; 满则关闭
        int slot = -1;
        for (int i = 0; i < TCP_OUT_MAX_CLIENTS; i++) {
            if (g_tcp.clients[i].fd < 0) { slot = i; break; }
        }
        if (slot < 0) {
            pthread_mutex_unlock(&g_tcp.lock);
            close(fd);
            continue;
        }

        tcp_client_t *c = &g_tcp.clients[slot];
        c->fd = fd;
        c->buf_size = (size_t)g_tcp.client_buf;
        c->buf_len = 0;
        c->buf = (char *)malloc(c->buf_size);
        if (!c->buf) {
            close(fd);
            c->fd = -1;
        } else {
            printf("[TCP] client %s:%d connected slot=%d\n",
                   inet_ntoa(cli.sin_addr), ntohs(cli.sin_port), slot);
        }
        pthread_mutex_unlock(&g_tcp.lock);
    }
    return NULL;
}

int tcp_output_init(const tcp_output_cfg_t *cfg)
{
    if (!cfg || !cfg->enable) return 0;

    memset(&g_tcp, 0, sizeof(g_tcp));
    g_tcp.enable = 1;
    g_tcp.port = (cfg->port > 0) ? cfg->port : 9000;
    g_tcp.max_clients = (cfg->max_clients > 0) ? cfg->max_clients : 4;
    g_tcp.client_buf = (cfg->client_buf > 0) ? cfg->client_buf : 65536;
    if (g_tcp.max_clients > TCP_OUT_MAX_CLIENTS) g_tcp.max_clients = TCP_OUT_MAX_CLIENTS;
    for (int i = 0; i < TCP_OUT_MAX_CLIENTS; i++) g_tcp.clients[i].fd = -1;

    pthread_mutex_init(&g_tcp.lock, NULL);

    g_tcp.listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_tcp.listen_fd < 0) {
        printf("[TCP] socket fail: %s\n", strerror(errno));
        return -1;
    }
    int yes = 1;
    setsockopt(g_tcp.listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)g_tcp.port);

    if (bind(g_tcp.listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        printf("[TCP] bind port %d fail: %s\n", g_tcp.port, strerror(errno));
        close(g_tcp.listen_fd);
        g_tcp.listen_fd = -1;
        return -1;
    }
    if (listen(g_tcp.listen_fd, g_tcp.max_clients) != 0) {
        printf("[TCP] listen fail: %s\n", strerror(errno));
        close(g_tcp.listen_fd);
        g_tcp.listen_fd = -1;
        return -1;
    }

    if (pthread_create(&g_tcp.accept_tid, NULL, accept_thread, NULL) != 0) {
        printf("[TCP] pthread_create accept fail\n");
        close(g_tcp.listen_fd);
        g_tcp.listen_fd = -1;
        return -1;
    }

    printf("[TCP] NDJSON output listening on port %d\n", g_tcp.port);
    return 0;
}

void tcp_output_deinit(void)
{
    g_tcp.enable = 0;
    if (g_tcp.listen_fd >= 0) {
        close(g_tcp.listen_fd);
        g_tcp.listen_fd = -1;
    }
    if (g_tcp.accept_tid) {
        pthread_join(g_tcp.accept_tid, NULL);
        g_tcp.accept_tid = 0;
    }
    pthread_mutex_lock(&g_tcp.lock);
    for (int i = 0; i < TCP_OUT_MAX_CLIENTS; i++)
        close_client_locked(i);
    pthread_mutex_unlock(&g_tcp.lock);
    pthread_mutex_destroy(&g_tcp.lock);
}

void tcp_output_send_line(const char *json_line)
{
    if (!g_tcp.enable || g_tcp.listen_fd < 0 || !json_line) return;
    size_t len = strlen(json_line);
    if (len == 0) return;

    // 分配临时行缓冲: json + '\n'
    char *line = (char *)malloc(len + 2);
    if (!line) return;
    memcpy(line, json_line, len);
    line[len] = '\n';
    line[len + 1] = '\0';

    pthread_mutex_lock(&g_tcp.lock);
    broadcast_locked(line, len + 1);
    pthread_mutex_unlock(&g_tcp.lock);
    free(line);
}

size_t tcp_output_json_escape(const char *str, char *dst, size_t dst_size)
{
    if (!str || !dst || dst_size == 0) return 0;
    size_t n = 0;
    const char *p = str;
    while (*p && n + 1 < dst_size) {
        unsigned char ch = (unsigned char)*p;
        switch (ch) {
        case '"':  dst[n++] = '\\'; dst[n++] = '"';  break;
        case '\\': dst[n++] = '\\'; dst[n++] = '\\'; break;
        case '\n': dst[n++] = '\\'; dst[n++] = 'n';  break;
        case '\r': dst[n++] = '\\'; dst[n++] = 'r';  break;
        case '\t': dst[n++] = '\\'; dst[n++] = 't';  break;
        default:
            if (ch < 0x20) {
                // 控制字符转成 \uXXXX (简单处理), 跳过不完整时截断
                if (n + 6 >= dst_size) { break; }
                char tmp[8];
                snprintf(tmp, sizeof(tmp), "\\u%04x", ch);
                for (int i = 0; tmp[i]; i++) dst[n++] = tmp[i];
            } else {
                dst[n++] = (char)ch;
            }
            break;
        }
        if (n >= dst_size) { dst_size = 0; break; }
    }
    dst[n] = '\0';
    return n;
}
