// =============================================================================
//  citradar_ab02.cpp
//  CitRadar-DX400LS-1126 接口协议 V2.1 —— AB02 视频目标模块 · 发送侧实现
//
//  编译（目标板 aarch64-linux 或任意 POSIX）:
//      g++ -std=c++14 -O2 -Wall -Wextra citradar_ab02.cpp -o ab02_send
//      ./ab02_send                 # 跑自测（含与 Python 参考实现的逐字节比对）
//
//  交叉编译（RV1126B，示例）:
//      aarch64-linux-gnu-g++ -std=c++14 -O2 -static \
//          citradar_ab02.cpp -o ab02_send
// =============================================================================

#include "citradar_ab02.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
#  define CITRADAR_HAVE_POSIX_SOCKET 1
#  include <arpa/inet.h>
#  include <fcntl.h>
#  include <netdb.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <sys/select.h>
#  include <sys/socket.h>
#  include <unistd.h>
#  ifndef MSG_NOSIGNAL
#    define MSG_NOSIGNAL 0
#  endif
#endif

namespace citradar {
namespace {

// ------------------------------------------------------------ 大端写入原语

inline void putU8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }

inline void putU16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    b.push_back(static_cast<uint8_t>(v & 0xFF));
}

// 负数经由 uint16_t 转换（模 2^16）后按位搬运，不依赖负数右移的实现定义行为
inline void putI16(std::vector<uint8_t>& b, int16_t v) {
    putU16(b, static_cast<uint16_t>(v));
}

// IEEE754 binary64 -> 8 字节大端；在大小端主机上结果一致
inline void putDouble(std::vector<uint8_t>& b, double v) {
    static_assert(sizeof(double) == 8, "需要 8 字节 double");
    uint64_t u = 0;
    std::memcpy(&u, &v, 8);
    for (int i = 7; i >= 0; --i) {
        b.push_back(static_cast<uint8_t>((u >> (i * 8)) & 0xFF));
    }
}

// 时间戳 8 字节：年-1900 / 月-1 / 日 / 时 / 分 / 秒 / 毫秒(u16 大端)
// 注意 struct tm 的 tm_year 本身即"距 1900 年数"、tm_mon 本身即 0 基，与协议约定天然一致
inline void putTimestamp(std::vector<uint8_t>& b, int64_t unix_ms) {
    time_t sec = static_cast<time_t>(unix_ms / 1000);
    int ms = static_cast<int>(unix_ms % 1000);
    if (ms < 0) { ms += 1000; --sec; }

    struct tm tmv;
    std::memset(&tmv, 0, sizeof(tmv));
#if defined(_WIN32)
    localtime_s(&tmv, &sec);
#else
    localtime_r(&sec, &tmv);
#endif
    b.push_back(static_cast<uint8_t>(tmv.tm_year & 0xFF));
    b.push_back(static_cast<uint8_t>(tmv.tm_mon & 0xFF));
    b.push_back(static_cast<uint8_t>(tmv.tm_mday & 0xFF));
    b.push_back(static_cast<uint8_t>(tmv.tm_hour & 0xFF));
    b.push_back(static_cast<uint8_t>(tmv.tm_min & 0xFF));
    b.push_back(static_cast<uint8_t>(tmv.tm_sec & 0xFF));
    putU16(b, static_cast<uint16_t>(ms));
}

// ------------------------------------------------------------ 车牌汉字映射
// 由协议表 5 的 36..71 顺序直接生成，UTF-8 字节用 \xNN 转义，源码保持纯 ASCII

struct PlateCnEntry { uint8_t index; char utf8[4]; };

const PlateCnEntry kPlateCnTable[] = {
    {36, "\xE4\xBA\xAC"},  // 京
    {37, "\xE6\xB2\xAA"},  // 沪
    {38, "\xE6\xB4\xA5"},  // 津
    {39, "\xE6\xB8\x9D"},  // 渝
    {40, "\xE5\x86\x80"},  // 冀
    {41, "\xE6\x99\x8B"},  // 晋
    {42, "\xE8\x92\x99"},  // 蒙
    {43, "\xE8\xBE\xBD"},  // 辽
    {44, "\xE5\x90\x89"},  // 吉
    {45, "\xE9\xBB\x91"},  // 黑
    {46, "\xE8\x8B\x8F"},  // 苏
    {47, "\xE6\xB5\x99"},  // 浙
    {48, "\xE7\x9A\x96"},  // 皖
    {49, "\xE9\x97\xBD"},  // 闽
    {50, "\xE8\xB5\xA3"},  // 赣
    {51, "\xE9\xB2\x81"},  // 鲁
    {52, "\xE8\xB1\xAB"},  // 豫
    {53, "\xE9\x84\x82"},  // 鄂
    {54, "\xE6\xB9\x98"},  // 湘
    {55, "\xE7\xB2\xA4"},  // 粤
    {56, "\xE7\x90\xBC"},  // 琼
    {57, "\xE5\xB7\x9D"},  // 川
    {58, "\xE8\xB4\xB5"},  // 贵
    {59, "\xE4\xBA\x91"},  // 云
    {60, "\xE8\x97\x8F"},  // 藏
    {61, "\xE9\x99\x95"},  // 陕
    {62, "\xE7\x94\x98"},  // 甘
    {63, "\xE9\x9D\x92"},  // 青
    {64, "\xE5\xAE\x81"},  // 宁
    {65, "\xE6\x96\xB0"},  // 新
    {66, "\xE8\xAD\xA6"},  // 警
    {67, "\xE4\xBD\xBF"},  // 使
    {68, "\xE9\xA2\x86"},  // 领
    {69, "\xE5\xAD\xA6"},  // 学
    {70, "\xE6\x8C\x82"},  // 挂
    {71, "\xE5\x86\x9B"},  // 军
};

// 返回下一个 UTF-8 序列的字节数
inline std::size_t utf8SeqLen(unsigned char c) {
    if (c < 0x80) return 1;
    if ((c & 0xE0) == 0xC0) return 2;
    if ((c & 0xF0) == 0xE0) return 3;
    if ((c & 0xF8) == 0xF0) return 4;
    return 1;
}

// 单个字符 -> 协议索引；无法映射返回 -1
inline int plateCharToIndex(const char* p, std::size_t len) {
    if (len == 1) {
        const unsigned char c = static_cast<unsigned char>(p[0]);
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'Z') return 10 + (c - 'A');
        if (c >= 'a' && c <= 'z') return 10 + (c - 'a');   // 容忍小写
        return -1;
    }
    for (std::size_t i = 0; i < sizeof(kPlateCnTable) / sizeof(kPlateCnTable[0]); ++i) {
        if (std::memcmp(kPlateCnTable[i].utf8, p, len) == 0) {
            return kPlateCnTable[i].index;
        }
    }
    return -1;
}

}  // namespace

// ------------------------------------------------------------ Ab02Encoder

Ab02Encoder::Ab02Encoder(const std::string& device_serial_hex) {
    dev_.fill(0);
    const std::size_t n = device_serial_hex.size() / 2;
    const std::size_t lim = n < kDevSerialLen ? n : kDevSerialLen;
    for (std::size_t i = 0; i < lim; ++i) {
        unsigned int b = 0;
        if (std::sscanf(device_serial_hex.c_str() + i * 2, "%2x", &b) != 1) break;
        dev_[i] = static_cast<uint8_t>(b);
    }
}

bool Ab02Encoder::encodePlate(const std::string& plate, uint8_t out[kPlateLen]) {
    std::memset(out, kPlatePad, kPlateLen);
    std::size_t ci = 0;
    for (std::size_t i = 0; i < plate.size() && ci < kPlateLen;) {
        const std::size_t n =
            utf8SeqLen(static_cast<unsigned char>(plate[i]));
        if (i + n > plate.size()) return false;   // UTF-8 被截断
        const int idx = plateCharToIndex(plate.c_str() + i, n);
        if (idx < 0) return false;                // 字符不在协议映射表内
        out[ci++] = static_cast<uint8_t>(idx);
        i += n;
    }
    return true;
}

std::string Ab02Encoder::decodePlate(const uint8_t in[kPlateLen]) {
    std::string s;
    for (std::size_t i = 0; i < kPlateLen; ++i) {
        const uint8_t v = in[i];
        if (v == kPlatePad) break;
        if (v <= 9) {
            s.push_back(static_cast<char>('0' + v));
        } else if (v <= 35) {
            s.push_back(static_cast<char>('A' + v - 10));
        } else {
            for (std::size_t k = 0; k < sizeof(kPlateCnTable) / sizeof(kPlateCnTable[0]); ++k) {
                if (kPlateCnTable[k].index == v) { s += kPlateCnTable[k].utf8; break; }
            }
        }
    }
    return s;
}

void Ab02Encoder::encodeTo(std::vector<uint8_t>& out,
                           const Ab02Target* targets, std::size_t count,
                           uint8_t serial, int64_t unix_ms) const {
    if (count > kMaxTargets) count = kMaxTargets;

    const std::size_t module_len = kModuleHead + kAb02Stride * count +
                                   (kModuleLenIncludesTail ? 1u : 0u);
    const std::size_t pkg_len = kHeaderSize + module_len + 1u;   // +1 为包尾 0xDD

    out.clear();
    out.reserve(pkg_len);

    // ---- 表 4 固定头部（32 字节）----
    putU16(out, kHeaderMagic);                              // 0  Header
    putU8 (out, kBodyType);                                 // 2  Body
    putU16(out, static_cast<uint16_t>(pkg_len));            // 3  Package length
    putU8 (out, serial);                                    // 5  Serial
    putU8 (out, kMsgPeriodic);                              // 6  MsgId
    putU8 (out, static_cast<uint8_t>(count));               // 7  Total Target
    putTimestamp(out, unix_ms);                             // 8  TimeStamp(8)
    out.insert(out.end(), dev_.begin(), dev_.end());        // 16 DeviceSerial(16)

    // ---- AB02 模块 ----
    putU16(out, kModVision);                                // 0  Fusion Type
    putU16(out, static_cast<uint16_t>(module_len));          // 2  Module Length
    putU8 (out, static_cast<uint8_t>(count));               // 4  Vision Num

    for (std::size_t i = 0; i < count; ++i) {
        const Ab02Target& t = targets[i];
        putU16(out, t.vision_id);                           // 5  Vision ID
        putU8 (out, t.lane_id);                             // 7  Lane ID
        putU8 (out, static_cast<uint8_t>(t.type));          // 8  Type
        putI16(out, t.dist_x);                              // 9  Distance_x
        putI16(out, t.dist_y);                              // 11 Distance_y
        putI16(out, t.speed_x);                             // 13 Speed_x (预留 0)
        putI16(out, t.speed_y);                             // 15 Speed_y (预留 0)
        putDouble(out, t.lon);                              // 17 Longitude
        putDouble(out, t.lat);                              // 25 Latitude
        putU16(out, t.box_landscape_x);                     // 33 Landscape x
        putU16(out, t.box_portrait_y);                      // 35 Portrait  y
        putU16(out, t.box_length);                          // 37 Box Length
        putU16(out, t.box_width);                           // 39 Box Width
        putU8 (out, static_cast<uint8_t>(t.veh_color));     // 41 Vehicle Color
        putU8 (out, t.plate_flag);                          // 42 Plate Flag
        putU16(out, t.plate_x);                             // 43 P.X
        putU16(out, t.plate_y);                             // 45 P.Y
        putU16(out, t.plate_len);                           // 47 P.L
        putU16(out, t.plate_wid);                           // 49 P.W
        putU8 (out, static_cast<uint8_t>(t.plate_color));   // 51 Plate Color
        uint8_t plate[kPlateLen];
        encodePlate(t.plate, plate);
        out.insert(out.end(), plate, plate + kPlateLen);    // 52 Plate Number(8)
    }

    out.push_back(kModuleTail);                             // 0xFF
    out.push_back(kPkgTail);                                // 0xDD
}

std::vector<uint8_t> Ab02Encoder::encode(const Ab02Target* targets,
                                         std::size_t count,
                                         uint8_t serial,
                                         int64_t unix_ms) const {
    std::vector<uint8_t> out;
    encodeTo(out, targets, count, serial, unix_ms);
    return out;
}

// ------------------------------------------------------------ TcpPublisher

#ifdef CITRADAR_HAVE_POSIX_SOCKET

namespace {

bool sendAll(int fd, const uint8_t* p, std::size_t n) {
    std::size_t off = 0;
    while (off < n) {
        const ssize_t w = ::send(fd, p + off, n - off, MSG_NOSIGNAL);
        if (w > 0) {
            off += static_cast<std::size_t>(w);
        } else if (w < 0 && errno == EINTR) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

bool connectWithTimeout(const std::string& host, uint16_t port,
                        int timeout_ms, int* out_fd) {
    char portstr[8];
    std::snprintf(portstr, sizeof(portstr), "%u", static_cast<unsigned>(port));

    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), portstr, &hints, &res) != 0) return false;

    const int fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) { freeaddrinfo(res); return false; }

    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    const int rc = ::connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);

    if (rc != 0 && errno != EINPROGRESS) { ::close(fd); return false; }

    if (rc != 0) {                              // 非阻塞 connect 进行中
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd, &wfds);
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        if (::select(fd + 1, nullptr, &wfds, nullptr, &tv) <= 0) {
            ::close(fd);
            return false;
        }
        int err = 0;
        socklen_t len = sizeof(err);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
            ::close(fd);
            return false;
        }
    }

    ::fcntl(fd, F_SETFL, flags);                // 恢复阻塞模式
    struct timeval snd;
    snd.tv_sec = 1;
    snd.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
    const int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    *out_fd = fd;
    return true;
}

}  // namespace

TcpPublisher::TcpPublisher(const Config& cfg)
    : cfg_(cfg), enc_(cfg.device_serial_hex) {}

TcpPublisher::~TcpPublisher() { close(); }

void TcpPublisher::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool TcpPublisher::ensureConnected() {
    if (fd_ >= 0) return true;
    return connectWithTimeout(cfg_.host, cfg_.port,
                              cfg_.connect_timeout_ms, &fd_);
}

bool TcpPublisher::send(const Ab02Target* targets, std::size_t count,
                        int64_t unix_ms) {
    enc_.encodeTo(buf_, targets, count, serial_, unix_ms);   // 复用缓冲，无堆分配
    if (!ensureConnected()) return false;
    if (!sendAll(fd_, buf_.data(), buf_.size())) {
        close();                                             // 下一帧触发重连
        return false;
    }
    serial_ = static_cast<uint8_t>((serial_ + 1) & 0xFF);     // 0..255 回绕
    ++frames_sent_;
    return true;
}

#else  // 非 POSIX 平台

TcpPublisher::TcpPublisher(const Config& cfg) : cfg_(cfg), enc_(cfg.device_serial_hex) {}
TcpPublisher::~TcpPublisher() {}
void TcpPublisher::close() {}
bool TcpPublisher::ensureConnected() { return false; }
bool TcpPublisher::send(const Ab02Target*, std::size_t, int64_t) { return false; }

#endif  // CITRADAR_HAVE_POSIX_SOCKET

}  // namespace citradar

// =============================================================================
//  自测：与 Python 参考实现（citradar_vision_only.py）逐字节比对
//  默认编译进可执行文件；若要把本文件当库链接，定义 CITRADAR_AB02_NO_MAIN
// =============================================================================

#if !defined(CITRADAR_AB02_NO_MAIN)

#include <iostream>

namespace {

const char* kExpectedHex =
    "aa539200cc0501037e08100e1e0000fafdc656bf7f00588d0000000000000000"
    "ab0200ab03006502050023fffa00000000000000000000000000000000000000"
    "0004b0032000b400dc060104e203c00050001a02370b0102030405ff00660108"
    "0034000b0000000000000000000000000000000000000000007802bc00f0017c"
    "0200000000000000000000ffffffffffffffff006700010008fff20000000000"
    "000000000000000000000000000000004001e0004400b4000000000000000000"
    "0000ffffffffffffffffffdd";

std::string toHex(const std::vector<uint8_t>& v) {
    static const char* d = "0123456789abcdef";
    std::string s;
    s.reserve(v.size() * 2);
    for (std::size_t i = 0; i < v.size(); ++i) {
        s.push_back(d[(v[i] >> 4) & 0xF]);
        s.push_back(d[v[i] & 0xF]);
    }
    return s;
}

int64_t localTimeToUnixMs(int y, int mo, int d, int h, int mi, int s, int ms) {
    struct tm tmv;
    std::memset(&tmv, 0, sizeof(tmv));
    tmv.tm_year = y - 1900;
    tmv.tm_mon = mo - 1;
    tmv.tm_mday = d;
    tmv.tm_hour = h;
    tmv.tm_min = mi;
    tmv.tm_sec = s;
    tmv.tm_isdst = -1;
    return static_cast<int64_t>(std::mktime(&tmv)) * 1000 + ms;
}

}  // namespace

int main() {
    using namespace citradar;
    int failures = 0;

    // ---- 构造与 Python 参考用例完全相同的 3 个目标 ----
    Ab02Target tg[3];

    tg[0].vision_id = 101;
    tg[0].lane_id = 2;
    tg[0].type = ObjType::Sedan;
    tg[0].dist_x = 35;
    tg[0].dist_y = -6;
    tg[0].setBox(1200, 800, 1380, 1020);
    tg[0].veh_color = VehColor::White;
    tg[0].plate_flag = 1;
    tg[0].setPlateBox(1250, 960, 1330, 986);
    tg[0].plate_color = PlateColor::Blue;
    tg[0].plate = "\xE7\xB2\xA4" "B12345";        // 粤B12345

    tg[1].vision_id = 102;
    tg[1].lane_id = 1;
    tg[1].type = ObjType::Truck;
    tg[1].dist_x = 52;
    tg[1].dist_y = 11;
    tg[1].setBox(120, 700, 360, 1080);
    tg[1].veh_color = VehColor::Blue;

    tg[2].vision_id = 103;
    tg[2].lane_id = 0;
    tg[2].type = ObjType::Pedestrian;
    tg[2].dist_x = 8;
    tg[2].dist_y = -14;
    tg[2].setBox(64, 480, 132, 660);
    tg[2].veh_color = VehColor::Other;

    Ab02Encoder enc("fdc656bf7f00588d");
    const int64_t ts = localTimeToUnixMs(2026, 9, 16, 14, 30, 0, 250);
    std::vector<uint8_t> pkt = enc.encode(tg, 3, 5, ts);

    std::cout << "报文长度: " << pkt.size() << " 字节 (期望 204)\n";
    if (pkt.size() != 204) { ++failures; }

    const std::string got = toHex(pkt);
    std::cout << "实际: " << got << "\n";
    std::cout << "期望: " << kExpectedHex << "\n";
    if (got == kExpectedHex) {
        std::cout << "[PASS] 与 Python 参考实现逐字节一致\n";
    } else {
        ++failures;
        std::cout << "[FAIL] 与参考实现不一致\n";
        for (std::size_t i = 0; i < got.size() && i < std::strlen(kExpectedHex); ++i) {
            if (got[i] != kExpectedHex[i]) {
                std::cout << "       首个差异在 hex 偏移 " << i
                          << " (字节 " << (i / 2) << "): 实际 " << got[i]
                          << " 期望 " << kExpectedHex[i] << "\n";
                break;
            }
        }
    }

    // ---- 结构自检 ----
    std::cout << "\n结构自检:\n";
    std::cout << "  头部前 8 字节: ";
    for (int i = 0; i < 8; ++i) std::printf("%02x ", pkt[i]);
    std::printf("\n");
    const std::size_t mod_off = kHeaderSize;
    const uint16_t mod_type = static_cast<uint16_t>((pkt[mod_off] << 8) | pkt[mod_off + 1]);
    const uint16_t mod_len = static_cast<uint16_t>((pkt[mod_off + 2] << 8) | pkt[mod_off + 3]);
    std::printf("  模块类型=0x%04X (期望 0xAB02)  模块长度=%u (期望 %u)\n",
                static_cast<unsigned>(mod_type), static_cast<unsigned>(mod_len),
                static_cast<unsigned>(kModuleHead + kAb02Stride * 3 + 1));
    if (mod_type != kModVision) ++failures;
    if (mod_len != kModuleHead + kAb02Stride * 3 + 1) ++failures;
    if (pkt.back() != kPkgTail) { ++failures; std::cout << "  [FAIL] 包尾非 0xDD\n"; }
    if (pkt[pkt.size() - 2] != kModuleTail) { ++failures; std::cout << "  [FAIL] 模块尾非 0xFF\n"; }

    // ---- 空目标帧 ----
    std::vector<uint8_t> empty = enc.encode(nullptr, 0, 6, ts);
    std::printf("\n空目标帧: %u 字节 (期望 39 = 32+5+1+1)\n",
                static_cast<unsigned>(empty.size()));
    if (empty.size() != 39) ++failures;

    // ---- 车牌编解码往返 ----
    std::cout << "\n车牌往返:\n";
    const char* plates[] = {"\xE7\xB2\xA4" "B12345", "\xE4\xBA\xAC" "A88888",
                            "\xE6\xB2\xAA" "AD12345"};
    const char* names[] = {"粤B12345", "京A88888", "沪AD12345"};
    for (int i = 0; i < 3; ++i) {
        uint8_t buf[kPlateLen];
        const bool ok = Ab02Encoder::encodePlate(plates[i], buf);
        const std::string back = Ab02Encoder::decodePlate(buf);
        const bool same = (back == plates[i]);
        std::printf("  [%s] %-10s -> ", (ok && same) ? "OK" : "NG", names[i]);
        for (int k = 0; k < 8; ++k) std::printf("%02x ", buf[k]);
        std::printf(" -> %s\n", back.c_str());
        if (!ok || !same) ++failures;
    }

    std::cout << "\n自测结果: " << (failures == 0 ? "全部通过" : "存在失败项") << "\n";
    return failures == 0 ? 0 : 1;
}

#endif  // !CITRADAR_AB02_NO_MAIN

// =============================================================================
//  现场需确认项（任一错都会导致平台解析异常）
//  A1  字节序：0xAA53 在链路上是 AA 53 还是 53 AA。本文件按大端实现。
//  A2  Module Length 是否含尾部 0xFF：本文件 kModuleLenIncludesTail = true。
//  A5  车牌不足 8 位的填充字节：本文件用 0xFF。
//  A8  识别框坐标系：3840x2160 原图还是 1280x720 子码流，是否做过缩放。
//  A9  Box Length / Box Width 谁是水平边：本文件按 长=水平边、宽=垂直边。
//  V1  Total Target 语义：本文件填"视觉目标数"（去重后的数量）。
//  V4  是否允许整包只含 AB02 而无 AB01。
//  V6  AB04 的"每分钟上报"是靠模块类型区分还是 MsgId 区分。
//
//  若平台确认"漏发 AB01 会报错"，只需把 TcpPublisher 组合成
//  "AB01 + AB02 同发"——AB02 的编码逻辑不用改，只是多调一个模块打包函数。
// =============================================================================
