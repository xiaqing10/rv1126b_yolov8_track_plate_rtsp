# yolo_web — Web 控制面板

基于纯 C 实现的车牌/车辆检测 Web 控制面板，运行于 RV1126B 板卡上，与 `yolo_rtsp` 配套使用。

## 特性

- **零框架**：纯 C POSIX socket 服务器，不依赖任何 Web 框架，仅需 `pthread` 与 `libdl`。
- **HTTP 服务**：监听 `8080` 端口，提供 HTML 控制页面与 REST API。
- **实时车牌推送**：通过 SSE（Server-Sent Events）将识别到的车牌实时推送到浏览器。
- **配置读写**：读取/写入 `/userdata/yolo/config/yolo.conf`，写入时保留注释。
- **自启动**：`run_yolo_rtsp.sh` 中集成自启动逻辑（通过 `killall -0` 幂等判断）。

## 接口

| 方法 | 路径 | 说明 |
|------|------|------|
| GET  | `/`            | HTML 控制页面 |
| GET  | `/api/state`   | 获取运行状态 |
| POST | `/api/start`   | 启动识别 |
| POST | `/api/stop`    | 停止识别 |
| POST | `/api/restart` | 重启识别 |
| POST | `/api/save`    | 保存配置 |
| GET  | `/api/plates`  | SSE，实时推送车牌 |

## 编译

在交叉编译环境（RV1126B SDK）下构建：

```sh
source /root/sdk/env-rv1126b.sh
cd /root/sdk/yolo_rtsp_app
make web/yolo_web
```

产物为 `web/yolo_web`（静态，无需 SDK 运行库）。

## 部署

将二进制拷贝到板卡并启动：

```sh
# 拷贝到板卡
scp web/yolo_web root@<board-ip>:/userdata/yolo/web/

# 板卡上启动
cd /userdata/yolo/web
nohup ./yolo_web > web.log 2>&1 &
```

浏览器访问 `http://<board-ip>:8080` 即可打开控制面板。

## 自启动

`run_yolo_rtsp.sh` 已包含自启动逻辑，脚本中通过 `killall -0 yolo_web` 判断进程是否已运行，实现幂等启动。
