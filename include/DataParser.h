#ifndef ARMOR_AIM_DATA_PARSER_H
#define ARMOR_AIM_DATA_PARSER_H

#include <array>
#include <cstdint>
#include <string>

#include "Common.h"

// 模拟接收: 解析 8 字节报文, 校验校验位, 还原数据
// 格式和 DataPacker 对应, 都是小端序
class DataParser {
public:
    explicit DataParser(double angle_scale = 100.0, double distance_scale = 1.0);

    // 解析 + 校验, 返回校验是否通过
    bool parse(const std::array<uint8_t, 8>& pkt, VisionData& data) const;

    std::string formatReceive(const std::array<uint8_t, 8>& pkt, bool ok,
                              const VisionData& data) const;
    void printReceive(const std::array<uint8_t, 8>& pkt, bool ok,
                      const VisionData& data) const;

private:
    double angle_scale_;
    double distance_scale_;
};

#endif // ARMOR_AIM_DATA_PARSER_H
