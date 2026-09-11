/*
 * yolo_web.c - RV1126B yolo_rtsp 网页控制守护进程
 *
 * 在板子上运行的轻量 HTTP 服务器, 提供:
 *   GET  /                   网页界面(HTML+JS 表单)
 *   GET  /api/state          返回 {running, config_text} JSON
 *   GET  /api/plates         SSE 实时推送检测到的车牌
 *   POST /api/save           保存配置(JSON: {config: "全文"}) 并可选重启
 *   POST /api/start          启动 yolo_rtsp
 *   POST /api/stop           停止 yolo_rtsp
 *   POST /api/restart        重启 yolo_rtsp (保存配置后)
 *
 * 单文件、无第三方依赖(仅标准 POSIX socket), 交叉编译到板端。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

#define PORT            8080
#define CONFIG_PATH     "/userdata/yolo/config/yolo.conf"
#define RUN_SCRIPT      "/userdata/yolo/run_yolo_rtsp.sh"
#define APP_DIR         "/userdata/yolo"

#define MAX_CFG         65536
#define MAX_PLATES      512
#define MAX_LINE        2048

/* 版本号: 编译时嵌入, 用于在 Web 页面核对板端运行的二进制新旧 */
#define WEB_VERSION     "v20260911-ROI-ReID"
#define WEB_BUILDTIME   __DATE__ " " __TIME__

/* ---------------- 配置管理 ---------------- */

static char g_cfg[MAX_CFG];

static int read_config(void) {
    FILE *f = fopen(CONFIG_PATH, "r");
    if (!f) { g_cfg[0] = 0; return -1; }
    size_t n = fread(g_cfg, 1, MAX_CFG - 1, f);
    fclose(f);
    g_cfg[n] = 0;
    return 0;
}

/* 写回完整配置文本 */
static int write_config(const char *text) {
    FILE *f = fopen(CONFIG_PATH, "w");
    if (!f) return -1;
    fputs(text, f);
    fclose(f);
    return 0;
}

/* ---------------- 进程管理 ---------------- */

static int g_app_running = 0;
static pthread_mutex_t g_app_mutex = PTHREAD_MUTEX_INITIALIZER;
static pid_t g_app_pid = -1;
static int g_stdout_pipe[2];

static int app_is_running(void) {
    if (g_app_pid > 0 && kill(g_app_pid, 0) == 0) return 1;
    if (system("busybox ps w | grep yolo_rtsp | grep -v grep >/dev/null 2>&1") == 0) return 1;
    return 0;
}

static void stop_app(void) {
    pthread_mutex_lock(&g_app_mutex);
    /* 先发 TERM 给 sh, 让 run_yolo_rtsp.sh 的 trap 清理 rkipc/子进程 */
    if (g_app_pid > 0) kill(g_app_pid, SIGTERM);
    /* 显式 kill yolo_rtsp 二进制 (它会忽略 TERM 时需升级) */
    system("killall yolo_rtsp 2>/dev/null; killall yolo_logrotate 2>/dev/null");
    for (int i = 0; i < 12; i++) {
        usleep(500000);
        /* 用 busybox ps 检测 yolo_rtsp 是否还在 */
        if (system("busybox ps w | grep yolo_rtsp | grep -v grep >/dev/null 2>&1") != 0) break;
    }
    system("killall -9 yolo_rtsp 2>/dev/null; killall -9 yolo_logrotate 2>/dev/null");
    if (g_app_pid > 0) kill(g_app_pid, SIGKILL);
    g_app_pid = -1;
    pthread_mutex_unlock(&g_app_mutex);
    g_app_running = 0;
}

/* ---------------- 车牌收集 ---------------- */

typedef struct {
    char line[MAX_LINE];
    time_t ts;
} plate_entry_t;

static plate_entry_t g_plates[MAX_PLATES];
static int g_plate_cnt = 0;
static pthread_mutex_t g_plate_mutex = PTHREAD_MUTEX_INITIALIZER;

static void plate_add(const char *line) {
    pthread_mutex_lock(&g_plate_mutex);
    if (g_plate_cnt < MAX_PLATES) {
        snprintf(g_plates[g_plate_cnt].line, MAX_LINE, "%s", line);
        g_plates[g_plate_cnt].ts = time(NULL);
        g_plate_cnt++;
    } else {
        /* 环形覆盖最旧 */
        memmove(&g_plates[0], &g_plates[1], sizeof(plate_entry_t) * (MAX_PLATES - 1));
        snprintf(g_plates[MAX_PLATES - 1].line, MAX_LINE, "%s", line);
        g_plates[MAX_PLATES - 1].ts = time(NULL);
    }
    pthread_mutex_unlock(&g_plate_mutex);
}

/* 日志文件路径 (与 yolo_logrotate.sh 保持一致) */
#define LOG_PATH        "/userdata/yolo/autostart.log"

/* 读取 app 输出: 追加到日志文件并解析 [PLATE] */
static void *app_output_thread(void *arg) {
    (void)arg;
    char buf[MAX_LINE];
    ssize_t n;
    size_t acc = 0;
    char linebuf[4096];
    FILE *lf = fopen(LOG_PATH, "a");
    if (lf) setvbuf(lf, NULL, _IONBF, 0);   /* 无缓冲, 保证网页实时可读 */
    while ((n = read(g_stdout_pipe[0], buf, sizeof(buf) - 1)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            if (buf[i] == '\n') {
                linebuf[acc] = 0;
                if (lf) { fputs(linebuf, lf); fputc('\n', lf); }
                if (strstr(linebuf, "[PLATE]")) plate_add(linebuf);
                acc = 0;
            } else if (acc < sizeof(linebuf) - 1) {
                linebuf[acc++] = buf[i];
            }
        }
    }
    if (lf) fclose(lf);
    return NULL;
}

/* ---------------- 启动 app ---------------- */

static int start_app(void) {
    pthread_mutex_lock(&g_app_mutex);
    if (g_app_pid > 0 && kill(g_app_pid, 0) == 0) {
        pthread_mutex_unlock(&g_app_mutex);
        return 0; /* 已在运行 */
    }
    pipe(g_stdout_pipe);
    pid_t pid = fork();
    if (pid == 0) {
        /* 子进程 */
        dup2(g_stdout_pipe[1], STDOUT_FILENO);
        dup2(g_stdout_pipe[1], STDERR_FILENO);
        close(g_stdout_pipe[0]);
        close(g_stdout_pipe[1]);
        /* 停止 rkipc */
        system("killall rkipc 2>/dev/null");
        execl("/bin/sh", "sh", RUN_SCRIPT, NULL);
        _exit(127);
    }
    close(g_stdout_pipe[1]);
    g_app_pid = pid;
    pthread_mutex_unlock(&g_app_mutex);
    g_app_running = 1;
    /* 启动输出读取线程 */
    pthread_t tid;
    pthread_create(&tid, NULL, app_output_thread, NULL);
    pthread_detach(tid);
    return 0;
}

/* ---------------- HTTP 工具 ---------------- */

static void url_decode(char *dst, const char *src) {
    while (*src) {
        if (*src == '%' && src[1] && src[2]) {
            char tmp[3] = { src[1], src[2], 0 };
            *dst++ = (char)strtol(tmp, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = 0;
}

static const char *http_date(void) {
    static char buf[64];
    time_t t = time(NULL);
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(buf, sizeof(buf), "%a, %d %b %Y %H:%M:%S GMT", &tm);
    return buf;
}

static void send_head(int c, int code, const char *ctype, long len) {
    char hdr[512];
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %ld\r\n"
        "Connection: close\r\n"
        "Date: %s\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n",
        code, code == 200 ? "OK" : (code == 404 ? "Not Found" : "Error"),
        ctype, len, http_date());
    write(c, hdr, strlen(hdr));
}

static void send_body(int c, const char *body) {
    write(c, body, strlen(body));
}

static const char *HTML_PAGE =
"<!DOCTYPE html><html lang=\"zh\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>F07 摄像头控制台</title><style>"
"*{box-sizing:border-box}"
"body{font-family:-apple-system,'Microsoft YaHei',sans-serif;margin:0;background:#eef1f5;color:#1f2d3d}"
".wrap{max-width:1000px;margin:0 auto;padding:20px}"
"h1{font-size:20px;margin:0 0 16px}"
".card{background:#fff;border:1px solid #e3e8ee;border-radius:10px;padding:18px;margin:14px 0;box-shadow:0 1px 2px rgba(0,0,0,.04)}"
"h2{font-size:15px;margin:0 0 14px;padding-bottom:8px;border-bottom:1px solid #eef1f5;color:#409eff}"
".row{display:flex;flex-wrap:wrap;gap:14px;margin:8px 0}"
".field{display:flex;flex-direction:column;flex:1;min-width:210px}"
"label{font-size:12px;color:#606a76;margin-bottom:5px}"
"input,select,textarea{font-size:14px;padding:8px 10px;border:1px solid #d5dbe3;border-radius:6px;background:#fafbfc;width:100%}"
"input:focus,select:focus{outline:none;border-color:#409eff;background:#fff}"
"textarea{font-family:monospace;font-size:12px;height:260px;resize:vertical;width:100%}"
"button{padding:9px 18px;border:none;border-radius:6px;cursor:pointer;font-size:14px;color:#fff}"
".btn-green{background:#22b573}.btn-red{background:#ef5350}.btn-blue{background:#3f8cf7}.btn-gray{background:#9aa5b1}"
"table{width:100%;border-collapse:collapse;font-size:13px}"
"th,td{border:1px solid #e3e8ee;padding:6px 10px;text-align:left}"
"th{background:#f5f7fa;color:#606a76;font-weight:600}"
".badge{display:inline-block;padding:3px 12px;border-radius:12px;font-size:12px;font-weight:600}"
".run{background:#22b573}.stop{background:#ef5350}"
".toolbar{display:flex;gap:10px;align-items:center;margin:6px 0}"
".tip{font-size:12px;color:#9099a5;margin-left:auto}"
"</style></head><body><div class=\"wrap\">"
"<h1>F07 摄像头控制台 <span style=\"font-size:13px;color:#9099a5;font-weight:normal\">" WEB_VERSION " (" WEB_BUILDTIME ")</span></h1>"

"<div class=\"card\"><h2>应用状态</h2>"
"<div class=\"toolbar\">"
"<span id=\"status\" class=\"badge stop\">未知</span>"
"<button class=\"btn-green\" onclick=\"startApp()\">启动</button>"
"<button class=\"btn-red\" onclick=\"stopApp()\">停止</button>"
"<button class=\"btn-blue\" onclick=\"restartApp()\">重启</button>"
"<span class=\"tip\">所有配置项与 yolo.conf 一一对应</span>"
"</div></div>"

"<div class=\"card\"><h2>检测模型</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>检测模型路径</label><input id=\"model\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>类别标签文件</label><input id=\"labels\" class=\"cfg\" type=\"text\"></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>类别数 (0=自动)</label><input id=\"class_num\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>检测置信度阈值</label><input id=\"conf_thresh\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>检测 NMS 阈值</label><input id=\"nms_thresh\" class=\"cfg\" type=\"text\"></div>"
"</div></div>"

"<div class=\"card\"><h2>跟踪器</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>跟踪算法</label><select id=\"tracker\" class=\"cfg\">"
"<option value=\"bytetrack\">bytetrack</option><option value=\"deepsort\">deepsort</option></select></div>"
"<div class=\"field\"><label>ReID 模型 (deepsort)</label><input id=\"reid_model\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>ReID 特征维数</label><input id=\"reid_dim\" class=\"cfg\" type=\"number\"></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>ReID 输入宽</label><input id=\"reid_w\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>ReID 输入高</label><input id=\"reid_h\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>ReID 推理间隔</label><input id=\"reid_interval\" class=\"cfg\" type=\"number\"></div>"
"</div></div>"

"<div class=\"card\"><h2>主码流与推流</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>宽</label><input id=\"width\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>高</label><input id=\"height\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>编码格式</label><select id=\"codec\" class=\"cfg\">"
"<option value=\"h264\">h264</option><option value=\"h265\">h265</option></select></div>"
"<div class=\"field\"><label>码率 (kbps)</label><input id=\"bitrate\" class=\"cfg\" type=\"number\"></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>RTSP 端口</label><input id=\"port\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>RTSP 路径</label><input id=\"path\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>推流开关</label><select id=\"push_stream\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"</div></div>"

"<div class=\"card\"><h2>车牌识别</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>启用</label><select id=\"car_enable\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"<div class=\"field\"><label>取帧通道</label><input id=\"car_chn\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>识别限频 (ms)</label><input id=\"car_interval_ms\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>识别引擎</label><select id=\"car_plate_type\" class=\"cfg\" onchange=\"plateTypeChanged()\">"
"<option value=\"rec\">rec (车牌+颜色)</option><option value=\"lprnet\">lprnet (仅车牌号)</option></select></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>车牌检测模型</label><input id=\"car_detect_model\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>牌照识别模型</label><input id=\"car_plate_model\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>车身颜色模型</label><input id=\"car_car_model\" class=\"cfg\" type=\"text\"></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>ROI 左上 X</label><input id=\"car_roi_x\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>ROI 左上 Y</label><input id=\"car_roi_y\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>ROI 宽</label><input id=\"car_roi_w\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>ROI 高</label><input id=\"car_roi_h\" class=\"cfg\" type=\"text\"></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>检测置信度</label><input id=\"car_conf_thresh\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>检测 NMS</label><input id=\"car_nms_thresh\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>车牌 NMS</label><input id=\"car_plate_nms_thresh\" class=\"cfg\" type=\"text\"></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>车牌计票阈值</label><input id=\"car_plate_vote\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>车色计票阈值</label><input id=\"car_color_vote\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>记录存活帧数</label><input id=\"car_track_age\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>车牌-车辆关联 IOU</label><input id=\"car_match_iou\" class=\"cfg\" type=\"text\"></div>"
"</div></div>"

"<div class=\"card\"><h2>OSD 显示</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>日期时间 OSD</label><select id=\"show_osd\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"<div class=\"field\"><label>帧率 OSD</label><select id=\"show_fps\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"<div class=\"field\"><label>车牌显示</label><select id=\"show_plate\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"</div>"
"<div class=\"row\">"
"<div class=\"field\"><label>文字放大倍数</label><input id=\"osd_scale\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>车牌文字放大</label><input id=\"osd_plate_scale\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>文字颜色</label><select id=\"osd_color\" class=\"cfg\">"
"<option value=\"white\">white</option><option value=\"black\">black</option><option value=\"red\">red</option>"
"<option value=\"green\">green</option><option value=\"blue\">blue</option><option value=\"yellow\">yellow</option>"
"<option value=\"cyan\">cyan</option><option value=\"magenta\">magenta</option></select></div>"
"</div></div>"

"<div class=\"card\"><h2>车牌图片保存</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>保存开关</label><select id=\"save_plate_image\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"<div class=\"field\"><label>保存目录</label><input id=\"plate_image_path\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>保存限频 (ms)</label><input id=\"plate_image_interval\" class=\"cfg\" type=\"number\"></div>"
"</div></div>"

"<div class=\"card\"><h2>主检测 ROI</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>X</label><input id=\"main_roi_x\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>Y</label><input id=\"main_roi_y\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>宽</label><input id=\"main_roi_w\" class=\"cfg\" type=\"text\"></div>"
"<div class=\"field\"><label>高</label><input id=\"main_roi_h\" class=\"cfg\" type=\"text\"></div>"
"</div></div>"

"<div class=\"card\"><h2>TCP 输出</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>TCP 输出开关</label><select id=\"tcp_enable\" class=\"cfg\">"
"<option value=\"1\">开</option><option value=\"0\">关</option></select></div>"
"<div class=\"field\"><label>端口</label><input id=\"tcp_port\" class=\"cfg\" type=\"number\"></div>"
"<div class=\"field\"><label>最大客户端数</label><input id=\"tcp_max_clients\" class=\"cfg\" type=\"number\"></div>"
"</div></div>"

"<div class=\"card\"><h2>日志</h2>"
"<div class=\"row\">"
"<div class=\"field\"><label>调试日志</label><select id=\"debug\" class=\"cfg\">"
"<option value=\"0\">关</option><option value=\"1\">开</option></select></div>"
"<div class=\"field\"><label>统计间隔 (秒, 0=禁用)</label><input id=\"print_interval_s\" class=\"cfg\" type=\"number\"></div>"
"</div>"
"<div class=\"toolbar\" style=\"margin-top:14px\">"
"<button class=\"btn-blue\" onclick=\"saveConfig()\">保存并重启</button>"
"<button class=\"btn-blue\" onclick=\"loadState()\">刷新配置</button>"
"</div></div>"

"<div class=\"card\"><h2>运行日志</h2>"
"<div class=\"toolbar\"><span class=\"tip\">日志文件: /userdata/yolo/autostart.log</span>"
"<button class=\"btn-blue\" onclick=\"loadLog()\">刷新日志</button></div>"
"<textarea id=\"logBox\" readonly></textarea>"
"</div>"

"<div class=\"card\"><h2>检测到的车牌</h2>"
"<table id=\"plateTable\"><thead><tr><th>时间</th><th>车牌</th></tr></thead>"
"<tbody id=\"plateBody\"><tr><td colspan=\"2\">暂无</td></tr></tbody></table>"
"</div>"

"<script>"
"var es=new EventSource('/api/plates');"
"es.onmessage=function(e){var d=JSON.parse(e.data);"
"var tb=document.getElementById('plateBody');"
"var tr=document.createElement('tr');"
"tr.innerHTML='<td>'+d.time+'</td><td>'+d.line+'</td>';"
"tb.insertBefore(tr,tb.firstChild);"
"while(tb.rows.length>50)tb.deleteRow(50);};"
"function setStatus(run){var s=document.getElementById('status');"
"s.className='badge '+(run?'run':'stop');s.textContent=run?'运行中':'已停止';}"
"function loadState(){fetch('/api/state').then(function(r){return r.json()}).then(function(d){"
"setStatus(d.running);"
"for(var k in d){if(k==='running')continue;var el=document.getElementById(k);if(el)el.value=d[k];}});}"
"function plateTypeChanged(){"
"var el=document.getElementById('car_plate_model');if(!el)return;"
"var t=document.getElementById('car_plate_type').value;"
"el.value=(t==='lprnet')?('/userdata/yolo/models/lprnet.rknn'):('/userdata/yolo/models/plate_rec_color_fp.rknn');}"
"function saveConfig(){"
"var b={};var els=document.querySelectorAll('.cfg');"
"for(var i=0;i<els.length;i++)b[els[i].id]=els[i].value;"
"fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})"
".then(function(r){return r.json()}).then(function(d){alert(d.msg)});}"
"function doPost(u){fetch(u,{method:'POST'}).then(function(r){return r.json()}).then(function(d){alert(d.msg);setTimeout(loadState,1500);});}"
"function startApp(){doPost('/api/start')}"
"function stopApp(){doPost('/api/stop')}"
"function restartApp(){doPost('/api/restart')}"
"function loadLog(){fetch('/api/log').then(function(r){return r.text()}).then(function(t){"
"var el=document.getElementById('logBox');if(el){el.value=t||'';el.scrollTop=el.scrollHeight;}});}"
"loadState();loadLog();setInterval(loadLog,3000);"
"</script></body></html>";

/* ---------------- 读取/解析配置为 JSON 片段 ---------------- */

/* 网页表单暴露的全部配置键 (与 yolo.conf 一致) */
static const char *CFG_FIELDS[] = {
    "model", "labels", "class_num", "conf_thresh", "nms_thresh",
    "tracker", "reid_model", "reid_dim", "reid_w", "reid_h", "reid_interval",
    "width", "height", "codec", "bitrate", "port", "path", "push_stream",
    "car_enable", "car_chn", "car_interval_ms", "car_plate_type",
    "car_detect_model", "car_plate_model", "car_car_model",
    "car_roi_x", "car_roi_y", "car_roi_w", "car_roi_h",
    "car_conf_thresh", "car_nms_thresh", "car_plate_nms_thresh",
    "car_plate_vote", "car_color_vote", "car_track_age", "car_match_iou",
    "show_osd", "show_fps", "osd_scale", "osd_plate_scale", "osd_color",
    "show_plate", "save_plate_image", "plate_image_path", "plate_image_interval",
    "main_roi_x", "main_roi_y", "main_roi_w", "main_roi_h",
    "tcp_enable", "tcp_port", "tcp_max_clients",
    "debug", "print_interval_s",
    NULL
};

static void cfg_value(const char *key, char *out, size_t outsz) {
    const char *p = g_cfg;
    out[0] = 0;
    while (p && *p) {
        /* 找到 key */
        const char *line = p;
        const char *nl = strchr(line, '\n');
        /* 跳过注释行和空行 */
        const char *s = line;
        while (*s == ' ' || *s == '\t') s++;
        if (*s != '#' && *s != ';' && *s != '\n' && *s != 0) {
            char kbuf[128], vbuf[1024];
            if (sscanf(line, "%127[^= \t] = %1023[^\n\r]", kbuf, vbuf) == 2) {
                /* 去掉 vbuf 尾部空白 */
                size_t vl = strlen(vbuf);
                while (vl && (vbuf[vl-1]==' '||vbuf[vl-1]=='\t'||vbuf[vl-1]=='\r')) vbuf[--vl]=0;
                /* 剥离行内注释 '#' / ';' */
                char *c = strchr(vbuf, '#');
                if (!c) c = strchr(vbuf, ';');
                if (c) { *c = 0; vl = strlen(vbuf); }
                if (!strcmp(kbuf, key)) {
                    snprintf(out, outsz, "%s", vbuf);
                    return;
                }
            }
        }
        p = nl ? nl + 1 : NULL;
    }
}

/* 更新配置中某个 key 的值, 保留注释与结构 */
static void cfg_set(const char *key, const char *value) {
    char newcfg[MAX_CFG];
    newcfg[0] = 0;
    const char *p = g_cfg;
    int done = 0;
    while (p && *p) {
        const char *line = p;
        const char *nl = strchr(line, '\n');
        size_t len = nl ? (size_t)(nl - line) : strlen(line);
        char buf[MAX_CFG];
        memcpy(buf, line, len);
        buf[len] = 0;
        const char *s = line;
        while (*s == ' ' || *s == '\t') s++;
        if (*s != '#' && *s != ';' && *s != '\n' && *s != 0) {
            char kbuf[128];
            if (sscanf(line, "%127[^= \t]", kbuf) == 1) {
                if (!strcmp(kbuf, key)) {
                    char repl[MAX_CFG];
                    snprintf(repl, sizeof(repl), "%s = %s\n", key, value);
                    strncat(newcfg, repl, MAX_CFG - strlen(newcfg) - 1);
                    done = 1;
                    p = nl ? nl + 1 : NULL;
                    continue;
                }
            }
        }
        strncat(newcfg, buf, MAX_CFG - strlen(newcfg) - 1);
        if (nl) strncat(newcfg, "\n", MAX_CFG - strlen(newcfg) - 1);
        p = nl ? nl + 1 : NULL;
    }
    if (!done) {
        char repl[MAX_CFG];
        snprintf(repl, sizeof(repl), "%s = %s\n", key, value);
        strncat(newcfg, repl, MAX_CFG - strlen(newcfg) - 1);
    }
    strncpy(g_cfg, newcfg, MAX_CFG - 1);
    g_cfg[MAX_CFG - 1] = 0;
}

/* ---------------- 日志读取 ---------------- */

#define LOG_TAIL_BYTES  65536   /* 网页一次最多读取日志尾部字节数 */

/* 读取日志尾部, 最多 tail 字节; 返回实际读到的内容(不含转义)。 */
static long read_log_tail(char *out, size_t outsz) {
    out[0] = 0;
    FILE *f = fopen(LOG_PATH, "r");
    if (!f) return 0;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return 0; }
    long sz = ftell(f);
    long start = sz > (long)outsz ? sz - (long)outsz : 0;
    if (start > 0 && fseek(f, start, SEEK_SET) != 0) { fclose(f); return 0; }
    size_t n = fread(out, 1, outsz - 1, f);
    out[n] = 0;
    fclose(f);
    return (long)n;
}

/* HTML 转义 (供日志文本安全显示) */

/* ---------------- 请求处理 ---------------- */

typedef struct {
    int fd;
} client_arg_t;
static void *handle_client(void *arg) {
    client_arg_t *a = (client_arg_t *)arg;
    int c = a->fd;
    free(a);
    /* 读完整 HTTP 请求: 按 Content-Length 循环收 body, 避免大 JSON(body 过长)
       或单次 read 未收全导致 /api/save 解析被截断、字段写不进 config */
    size_t reqcap = 8192, reqlen = 0;
    char *req = malloc(reqcap + 1);
    if (!req) { close(c); return NULL; }
    long clen = -1;
    for (;;) {
        if (reqlen + 4096 > reqcap) {
            reqcap *= 2;
            char *nr = realloc(req, reqcap + 1);
            if (!nr) { free(req); close(c); return NULL; }
            req = nr;
        }
        ssize_t n = read(c, req + reqlen, reqcap - reqlen);
        if (n <= 0) break;
        reqlen += (size_t)n;
        req[reqlen] = 0;
        /* 已收到完整 header */
        char *hdr_end = strstr(req, "\r\n\r\n");
        if (!hdr_end) {
            /* header 未收全继续读 */
            if (reqlen >= 65536) break;   /* 防御: header 异常过大 */
            continue;
        }
        if (clen < 0) {
            /* 解析 Content-Length */
            char *hp = req;
            while ((hp = strstr(hp, "\r\n")) != NULL && hp < hdr_end) {
                char line[128];
                const char *ls = hp + 2;
                const char *le = strstr(ls, "\r\n");
                if (!le) break;
                size_t lsz = (le - ls < 120) ? (size_t)(le - ls) : 120;
                memcpy(line, ls, lsz);
                line[lsz] = 0;
                if (strncasecmp(line, "Content-Length:", 15) == 0) {
                    clen = atol(line + 15);
                    break;
                }
                hp = (char *)le;
            }
        }
        if (clen >= 0) {
            size_t body_start = (size_t)(hdr_end - req) + 4;
            if (reqlen >= body_start + (size_t)clen) break;   /* body 收全 */
        } else {
            /* 无 Content-Length 头 = 无 body (GET / SSE 长连接等), header 收全即止 */
            break;
        }
    }
    req[reqlen] = 0;
    /* 解析首行 */
    char method[16], path[1024];
    sscanf(req, "%15s %1023s", method, path);

    if (!strcmp(path, "/api/plates")) {
        /* SSE 长连接 */
        char hdr[512];
        snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
            "Cache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n");
        write(c, hdr, strlen(hdr));
        /* 推送历史 + 持续等待新数据 */
        int last = g_plate_cnt;
        for (;;) {
            pthread_mutex_lock(&g_plate_mutex);
            for (int i = last; i < g_plate_cnt; i++) {
                char evt[MAX_LINE + 64];
                snprintf(evt, sizeof(evt),
                    "data: {\"line\":\"%s\",\"time\":\"%ld\"}\n\n",
                    g_plates[i].line, g_plates[i].ts);
                write(c, evt, strlen(evt));
            }
            last = g_plate_cnt;
            pthread_mutex_unlock(&g_plate_mutex);
            usleep(500000);
        }
    }
    else if (!strcmp(path, "/api/state")) {
        char json[MAX_CFG + 2048];
        char v[1024];
        snprintf(json, sizeof(json), "{\"running\":%d", app_is_running());
        for (int i = 0; CFG_FIELDS[i]; i++) {
            cfg_value(CFG_FIELDS[i], v, sizeof(v));
            strncat(json, ",\"", sizeof(json) - strlen(json) - 1);
            strncat(json, CFG_FIELDS[i], sizeof(json) - strlen(json) - 1);
            strncat(json, "\":\"", sizeof(json) - strlen(json) - 1);
            strncat(json, v, sizeof(json) - strlen(json) - 1);
            strncat(json, "\"", sizeof(json) - strlen(json) - 1);
        }
        strncat(json, "}", sizeof(json) - strlen(json) - 1);
        send_head(c, 200, "application/json", strlen(json));
        send_body(c, json);
    }
    else if (!strcmp(method, "POST") && !strcmp(path, "/api/save")) {
        /* 解析 JSON body 中的各字段 */
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { send_head(c, 400, "text/plain", 4); send_body(c, "bad"); free(req); close(c); return NULL; }
        body += 4;
        /* 提取 key:value 简单解析 */
        char key[64], val[256];
        const char *p = body;
        read_config();
        while (*p) {
            p = strchr(p, '"');
            if (!p) break;
            p++;
            int k = 0;
            while (*p && *p != '"' && k < 63) key[k++] = *p++;
            key[k] = 0;
            if (*p) p++;
            while (*p && (*p == ':' || *p == ' ' || *p == '\t')) p++;
            if (*p == '"') {
                p++;
                int v = 0;
                while (*p && *p != '"' && v < 255) val[v++] = *p++;
                val[v] = 0;
                if (*p) p++;
                /* 更新 */
                if (key[0]) cfg_set(key, val);
            } else if (*p == '{' || *p == '[') {
                /* 跳过嵌套 - 本项目没有 */
            }
        }
        write_config(g_cfg);
        /* 重启应用 */
        stop_app();
        start_app();
        const char *resp = "{\"msg\":\"已保存并重启\"}";
        send_head(c, 200, "application/json", strlen(resp));
        send_body(c, resp);
    }
    else if (!strcmp(method, "POST") && !strcmp(path, "/api/start")) {
        start_app();
        const char *resp = "{\"msg\":\"启动中\"}";
        send_head(c, 200, "application/json", strlen(resp));
        send_body(c, resp);
    }
    else if (!strcmp(method, "POST") && !strcmp(path, "/api/stop")) {
        stop_app();
        const char *resp = "{\"msg\":\"已停止\"}";
        send_head(c, 200, "application/json", strlen(resp));
        send_body(c, resp);
    }
    else if (!strcmp(method, "POST") && !strcmp(path, "/api/restart")) {
        stop_app();
        start_app();
        const char *resp = "{\"msg\":\"已重启\"}";
        send_head(c, 200, "application/json", strlen(resp));
        send_body(c, resp);
    }
    else if (!strcmp(path, "/api/log")) {
        /* 返回日志尾部 (纯文本, 避免 JSON 字符串中的换行/转义问题) */
        static char raw[LOG_TAIL_BYTES + 1];
        long n = read_log_tail(raw, sizeof(raw));
        send_head(c, 200, "text/plain; charset=utf-8", n);
        send_body(c, raw);
    }
    else {
        /* 默认: 网页 */
        send_head(c, 200, "text/html; charset=utf-8", strlen(HTML_PAGE));
        send_body(c, HTML_PAGE);
    }
    free(req);
    close(c);
    return NULL;
}

int main(void) {
    signal(SIGPIPE, SIG_IGN);
    signal(SIGCHLD, SIG_IGN);
    read_config();

    int s = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    /* 关键: 设 FD_CLOEXEC, 避免 fork 出的子进程(run_yolo_rtsp.sh / logrotate)
       继承监听 socket fd, 否则父进程被杀后端口仍被占用 */
    fcntl(s, F_SETFD, fcntl(s, F_GETFD, 0) | FD_CLOEXEC);
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(PORT);
    if (bind(s, (struct sockaddr *)&a, sizeof(a)) < 0) {
        fprintf(stderr, "bind: %s\n", strerror(errno));
        return 1;
    }
    if (listen(s, 16) < 0) { perror("listen"); return 1; }
    fprintf(stderr, "yolo_web listening on %d\n", PORT);

    for (;;) {
        int c = accept(s, NULL, NULL);
        if (c < 0) continue;
        /* 每个连接一个线程 */
        client_arg_t *a = malloc(sizeof(client_arg_t));
        a->fd = c;
        pthread_t tid;
        pthread_create(&tid, NULL, handle_client, a);
        pthread_detach(tid);
    }
    return 0;
}
