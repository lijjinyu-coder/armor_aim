#ifndef ARMOR_AIM_DATA_PACKER_H
#define ARMOR_AIM_DATA_PACKER_H

#include <array>
#include <cstdint>
#include <string>

#include "Common.h"

// 模拟发送: 把 yaw / pitch / distance / 状态打包成 8 字节报文
// 格式(小端序):
//   0-1: yaw      int16   度 * 100
//   2-3: pitch    int16   度 * 100
//   4-5: distance uint16  mm
//   6:   状态     uint8
//   7:   校验位   = 前 7 字节相加取低 8 位
class DataPacker {
public:
    explicit DataPacker(double angle_scale = 100.0, double distance_scale = 1.0);

    std::array<uint8_t, 8> pack(const VisionData& data) const;

    std::string formatSend(const std::array<uint8_t, 8>& pkt) const;
    void printSend(const std::array<uint8_t, 8>& pkt) const;

private:
    double angle_scale_;
    double distance_scale_;
};

#endif // ARMOR_AIM_DATA_PACKER_H
