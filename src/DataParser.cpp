#include "DataParser.h"

#include <iomanip>
#include <iostream>
#include <sstream>

DataParser::DataParser(double angle_scale, double distance_scale)
    : angle_scale_(angle_scale), distance_scale_(distance_scale) {}

bool DataParser::parse(const std::array<uint8_t, 8>& pkt, VisionData& data) const {
    // 小端序拼回整数
    int16_t yaw_i = static_cast<int16_t>(
        static_cast<uint16_t>(pkt[0]) | (static_cast<uint16_t>(pkt[1]) << 8));
    int16_t pitch_i = static_cast<int16_t>(
        static_cast<uint16_t>(pkt[2]) | (static_cast<uint16_t>(pkt[3]) << 8));
    uint16_t dist_i = static_cast<uint16_t>(pkt[4]) |
                      (static_cast<uint16_t>(pkt[5]) << 8);

    // 校验: 前 7 个字节相加, 和最后一字节比对
    uint8_t sum = 0;
    for (int i = 0; i < 7; ++i) sum += pkt[i];
    bool checksum_ok = (sum == pkt[7]);

    // 除以缩放系数还原
    data.yaw_deg     = static_cast<float>(yaw_i) / static_cast<float>(angle_scale_);
    data.pitch_deg   = static_cast<float>(pitch_i) / static_cast<float>(angle_scale_);
    data.distance_mm = static_cast<float>(dist_i) / static_cast<float>(distance_scale_);
    data.state       = static_cast<TargetState>(pkt[6]);
    return checksum_ok;
}

std::string DataParser::formatReceive(const std::array<uint8_t, 8>& pkt, bool ok,
                                      const VisionData& data) const {
    (void)pkt; // 这里用不到报文本身
    std::ostringstream oss;
    oss << "收到: yaw=" << std::fixed << std::setprecision(2) << data.yaw_deg
        << " deg, pitch=" << data.pitch_deg << " deg, distance=" << data.distance_mm
        << " mm, state=" << stateToString(data.state)
        << ", checksum=" << (ok ? "OK" : "FAIL");
    return oss.str();
}

void DataParser::printReceive(const std::array<uint8_t, 8>& pkt, bool ok,
                              const VisionData& data) const {
    std::cout << formatReceive(pkt, ok, data) << std::endl;
}
