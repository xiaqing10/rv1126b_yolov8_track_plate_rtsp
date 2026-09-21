// =============================================================================
//  citradar_ab02.hpp
//  CitRadar-DX400LS-1126 接口协议 V2.1 —— AB02 视频目标模块 · 发送侧封装
//  纯视觉方案：只上报 AB02，不涉及雷达
//
//  文档依据: DGL-PV2003-20240810 / 接口协议 V2.1 (2025-11-24)
//  报文体量: 头部 32B + 模块头 5B + 55B×n + 模块尾 0xFF + 包尾 0xDD
//
//  ── 三层封装，职责严格分离 ────────────────────────────────────────────────
//    Ab02Target    数据层 —— 算法侧只填这个 POD，与协议字节布局彻底解耦
//    Ab02Encoder   协议层 —— 纯计算，无 I/O、无全局状态、无动态分配依赖，可单测
//    TcpPublisher  传输层 —— socket / 断线重连 / 50ms 节拍 / 序列号独占持有
//
//  ── 两条铁律 ──────────────────────────────────────────────────────────────
//    1) 绝不用 memcpy 直接序列化结构体：会踩内存对齐填充（sizeof 可能变 56/60）
//       与主机字节序，必须逐字段显式写大端。
//    2) 每帧不要堆分配：用 encodeTo(out, ...) 复用缓冲，out.reserve(1024)。
//
//  编译: g++ -std=c++14 -O2 -Wall -Wextra citradar_ab02.cpp -o ab02_send
// =============================================================================

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace citradar {

// ---------------------------------------------------------------- 协议常量

constexpr uint16_t kHeaderMagic  = 0xAA53;
constexpr uint8_t  kBodyType     = 0x92;
constexpr uint8_t  kModuleTail   = 0xFF;
constexpr uint8_t  kPkgTail      = 0xDD;

constexpr uint16_t kModVision    = 0xAB02;   // 视频目标模块

constexpr std::size_t kHeaderSize  = 32;     // 表 4 固定头部
constexpr std::size_t kModuleHead  = 5;      // Type(2) + Len(2) + Num(1)
constexpr std::size_t kAb02Stride  = 55;     // 每个视频目标的定长
constexpr std::size_t kMaxTargets  = 255;
constexpr std::size_t kDevSerialLen = 16;
constexpr std::size_t kPlateLen     = 8;

static_assert(kHeaderSize == 2 + 1 + 2 + 1 + 1 + 1 + 8 + 16,
              "头部长度必须为 32 字节");
static_assert(kAb02Stride ==
              2 + 1 + 1 + 2 + 2 + 2 + 2 + 8 + 8 + 2 + 2 + 2 + 2 + 1 + 1 +
                  2 + 2 + 2 + 2 + 1 + 8,
              "AB02 单项必须为 55 字节");

// Module Length 是否包含模块尾部的 0xFF（AMBIGUITY_A2，抓包确认后改这里）
constexpr bool kModuleLenIncludesTail = true;

// 车牌不足 8 位时的填充字节（AMBIGUITY_A5）
constexpr uint8_t kPlatePad = 0xFF;

// 周期上报（表 4 的 Message Identification）
constexpr uint8_t kMsgPeriodic = 0x01;

// ---------------------------------------------------------------- 映射表枚举
// 数值必须与协议表 5 逐字对应，不得按印象改写

enum class ObjType : uint8_t {
    Other = 0, Pedestrian = 1, Bicycle = 2, Ebike = 3, Tricycle = 4,
    Sedan = 5, Bus = 6, Van = 7, Truck = 8, Suv = 9, Mpv = 10,
    SmallTruck = 11, BigTruck = 12, LightBus = 13, SmallBus = 14,
    BigBus = 15, MicroVan = 16, Pickup = 17, Trailer = 18, Mixer = 19,
    Tanker = 20, Crane = 21, Fire = 22, DumpTruck = 23, Escort = 24,
    Repair = 25, Rescue = 26, BoardTruck = 27
};

enum class VehColor : uint8_t {
    Other = 0, Black = 1, Blue = 2, Brown = 3, Green = 4, Silver = 5,
    White = 6, Red = 7, Yellow = 8, Pink = 9, Purple = 10, Cyan = 11,
    Gray = 12, Golden = 13
};

enum class PlateColor : uint8_t {
    Other = 0, Yellow = 1, Blue = 2, Black = 3, White = 4,
    Green = 5, YellowGreen = 6, GradientGreen = 7
};

// ---------------------------------------------------------------- 数据层

struct Ab02Target {
    // --- 视觉身份与分类 ---
    uint16_t vision_id = 0;                 // 跟踪 ID，需跨帧稳定
    uint8_t  lane_id   = 0;                 // 车道号；无车道线配置时填 0
    ObjType  type      = ObjType::Other;

    // --- 世界坐标与速度（AB02 语义：图像转世界坐标） ---
    int16_t dist_x  = 0;                    // 米。需先做单应/标定，勿填像素
    int16_t dist_y  = 0;
    int16_t speed_x = 0;                    // AB02 为预留字段，固定填 0
    int16_t speed_y = 0;
    double  lon     = 0.0;                  // 无相机 GPS 时填 0
    double  lat     = 0.0;

    // --- 识别框（像素，坐标系见 AMBIGUITY_A8） ---
    uint16_t box_landscape_x = 0;           // 协议: Landscape Coordinates
    uint16_t box_portrait_y  = 0;           // 协议: Portrait Coordinates
    uint16_t box_length      = 0;           // 协议: Box Length
    uint16_t box_width       = 0;           // 协议: Box Width

    // --- 车辆属性 ---
    VehColor veh_color = VehColor::Other;

    // --- 车牌 ---
    uint8_t    plate_flag = 0;              // 0 未识别 / 1 识别
    uint16_t   plate_x = 0, plate_y = 0;
    uint16_t   plate_len = 0, plate_wid = 0;
    PlateColor plate_color = PlateColor::Other;
    std::string plate;                      // UTF-8，如 "粤B12345"；空表示无

    // 便捷构造：像素框 (x1,y1,x2,y2) -> 协议四字段
    void setBox(int x1, int y1, int x2, int y2) {
        box_landscape_x = static_cast<uint16_t>(x1);
        box_portrait_y  = static_cast<uint16_t>(y1);
        box_length      = static_cast<uint16_t>(x2 - x1);   // 长 = 水平边
        box_width       = static_cast<uint16_t>(y2 - y1);   // 宽 = 垂直边
    }
    void setPlateBox(int x1, int y1, int x2, int y2) {
        plate_x   = static_cast<uint16_t>(x1);
        plate_y   = static_cast<uint16_t>(y1);
        plate_len = static_cast<uint16_t>(x2 - x1);
        plate_wid = static_cast<uint16_t>(y2 - y1);
    }
};

// ---------------------------------------------------------------- 协议层

class Ab02Encoder {
public:
    // device_serial_hex: 16 字节设备序列号的十六进制串，如 "fdc656bf7f00588d"
    explicit Ab02Encoder(const std::string& device_serial_hex = "fdc656bf7f00588d");

    // 组一个"只含 AB02"的完整报文
    //   targets/count : 视觉目标（count 上限 255，超出截断）
    //   serial        : 包序列号 0..255，由调用方（发送器）独占维护
    //   unix_ms       : UTC 毫秒时间戳，内部按本地时区拆成协议字段
    std::vector<uint8_t> encode(const Ab02Target* targets, std::size_t count,
                                uint8_t serial, int64_t unix_ms) const;

    // 复用缓冲版本，避免每帧堆分配（推荐在推理循环里用这个）
    void encodeTo(std::vector<uint8_t>& out,
                  const Ab02Target* targets, std::size_t count,
                  uint8_t serial, int64_t unix_ms) const;

    // 车牌编解码（供单测与交叉验证）
    static bool encodePlate(const std::string& plate, uint8_t out[kPlateLen]);
    static std::string decodePlate(const uint8_t in[kPlateLen]);

    const std::array<uint8_t, kDevSerialLen>& deviceSerial() const { return dev_; }

private:
    std::array<uint8_t, kDevSerialLen> dev_{};
};

// ---------------------------------------------------------------- 传输层

class TcpPublisher {
public:
    struct Config {
        std::string host = "192.168.1.123";
        uint16_t    port = 9999;
        int         interval_ms  = 50;      // 协议默认周期 50ms
        int         reconnect_ms = 3000;
        int         connect_timeout_ms = 2000;
        std::string device_serial_hex = "fdc656bf7f00588d";
    };

    explicit TcpPublisher(const Config& cfg);

    // 阻塞发一帧。内部自动维护序列号自增与回绕（0..255）。
    // 返回 false 表示本帧未发出（链路断开且重连失败），不抛异常。
    bool send(const Ab02Target* targets, std::size_t count, int64_t unix_ms);

    uint8_t serial() const { return serial_; }
    uint64_t framesSent() const { return frames_sent_; }
    bool connected() const { return fd_ >= 0; }

    void close();
    ~TcpPublisher();

private:
    bool ensureConnected();

    Config  cfg_;
    Ab02Encoder enc_;
    uint8_t  serial_ = 0;
    uint64_t frames_sent_ = 0;
    int      fd_ = -1;
    std::vector<uint8_t> buf_;   // 复用缓冲，避免每帧堆分配
};

}  // namespace citradar
