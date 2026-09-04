#include "DataPacker.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

// 报文长度
const int MSG_LEN = 8;

DataPacker::DataPacker(double angle_scale, double distance_scale)
    : angle_scale_(angle_scale), distance_scale_(distance_scale) {}

std::array<uint8_t, 8> DataPacker::pack(const VisionData& data) const {
    std::array<uint8_t, 8> pkt{};

    // yaw/pitch: 度 * 100 -> int16, 超出范围就截断
    long yaw_i   = std::lround(data.yaw_deg * angle_scale_);
    long pitch_i = std::lround(data.pitch_deg * angle_scale_);
    if (yaw_i > 32767) yaw_i = 32767;
    if (yaw_i < -32768) yaw_i = -32768;
    if (pitch_i > 32767) pitch_i = 32767;
    if (pitch_i < -32768) pitch_i = -32768;
    int16_t yaw   = static_cast<int16_t>(yaw_i);
    int16_t pitch = static_cast<int16_t>(pitch_i);

    // distance: mm -> uint16, 负数按 0
    long dist_i = std::lround(data.distance_mm * distance_scale_);
    if (dist_i > 65535) dist_i = 65535;
    if (dist_i < 0) dist_i = 0;
    uint16_t dist = static_cast<uint16_t>(dist_i);

    // 小端序写入字节
    pkt[0] = static_cast<uint8_t>(yaw & 0xFF);
    pkt[1] = static_cast<uint8_t>((yaw >> 8) & 0xFF);
    pkt[2] = static_cast<uint8_t>(pitch & 0xFF);
    pkt[3] = static_cast<uint8_t>((pitch >> 8) & 0xFF);
    pkt[4] = static_cast<uint8_t>(dist & 0xFF);
    pkt[5] = static_cast<uint8_t>((dist >> 8) & 0xFF);
    pkt[6] = static_cast<uint8_t>(data.state);

    // 校验位 = 前 7 个字节相加, 取低 8 位
    uint8_t sum = 0;
    for (int i = 0; i < 7; ++i) sum += pkt[i];
    pkt[7] = sum;
    return pkt;
}

std::string DataPacker::formatSend(const std::array<uint8_t, 8>& pkt) const {
    std::ostringstream oss;
    oss << "发送: ";
    for (int i = 0; i < MSG_LEN; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pkt[i]);
        if (i + 1 < MSG_LEN) oss << " ";
    }
    return oss.str();
}

void DataPacker::printSend(const std::array<uint8_t, 8>& pkt) const {
    std::cout << formatSend(pkt) << std::endl;
}
