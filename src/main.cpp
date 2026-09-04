#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

#include <opencv2/highgui.hpp>

#include "ArmorDetector.h"
#include "ConfigLoader.h"
#include "DataPacker.h"
#include "DataParser.h"
#include "DebugDrawer.h"
#include "PoseSolver.h"
#include "TargetSelector.h"
#include "VideoInput.h"

static void printUsage() {
    std::cout
        << "用法:\n"
        << "  armor_aim                    读 config/config.yaml 里的视频, 跑完整流程\n"
        << "  armor_aim --video <路径>     临时换一个测试视频\n"
        << "  armor_aim --config <路径>    指定配置文件\n"
        << "运行中按键: Q 或 ESC 退出\n";
}

// 找 config/config.yaml, 也支持 --config 指定
static std::string findConfigPath(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) return argv[i + 1];
    }
    const char* candidates[] = {"config/config.yaml",
                                "../config/config.yaml",
                                "../../config/config.yaml"};
    for (const char* c : candidates) {
        if (std::ifstream(c).good()) return c;
    }
    return "config/config.yaml";
}

static std::string defaultOr(const std::string& v, const char* dflt) {
    return v.empty() ? std::string(dflt) : v;
}

int runPipeline(const AppConfig& cfg) {
    // 视频路径没填就提示退出
    if (cfg.video_path.empty()) {
        std::cerr << "错误: 还没填 video_path!\n"
                  << "请打开 config/config.yaml, 把 video_path 填成你的测试视频,\n"
                  << "或者运行时加参数, 例如:\n"
                  << "  armor_aim --video videos/xxx.mp4\n";
        return 1;
    }

    // 打开视频
    VideoInput input;
    if (!input.open(cfg.video_path)) return 1;

    // 准备好各个模块
    ArmorDetector detector;
    ArmorDetector::Params det_params;
    det_params.enemy_color      = cfg.enemy_color;
    det_params.preprocess_mode  = cfg.preprocess_mode;
    det_params.h_low = cfg.h_low; det_params.s_low = cfg.s_low; det_params.v_low = cfg.v_low;
    det_params.h_high = cfg.h_high; det_params.s_high = cfg.s_high; det_params.v_high = cfg.v_high;
    det_params.gray_thresh     = cfg.gray_thresh;
    det_params.bgr_diff_thresh = cfg.bgr_diff_thresh;
    det_params.lightbar_min_area  = cfg.lightbar_min_area;
    det_params.lightbar_min_ratio = cfg.lightbar_min_ratio;
    det_params.lightbar_max_angle = cfg.lightbar_max_angle;
    det_params.armor_min_ratio         = cfg.armor_min_ratio;
    det_params.armor_max_ratio         = cfg.armor_max_ratio;
    det_params.armor_angle_sum_max     = cfg.armor_angle_sum_max;
    det_params.armor_length_diff_ratio = cfg.armor_length_diff_ratio;
    det_params.armor_y_offset_ratio    = cfg.armor_y_offset_ratio;
    detector.setParams(det_params);

    TargetSelector selector(cfg.lost_threshold);

    PoseSolver solver;
    PoseSolver::Params pose_params;
    pose_params.armor_width_mm  = cfg.armor_width_mm;
    pose_params.armor_height_mm = cfg.armor_height_mm;
    solver.setParams(pose_params);

    DebugDrawer drawer;

    // 打包 / 解析报文用
    DataPacker packer;
    DataParser parser;

    // 报文记录文件(阶段二验收用)
    try { std::filesystem::create_directories("output"); } catch (...) {}
    std::ofstream comm_log("output/comm.log", std::ios::app);
    if (!comm_log) std::cerr << "报文记录文件打开失败: output/comm.log" << std::endl;

    // 结果视频(保存运行结果)
    std::string out_path = defaultOr(cfg.save_video_path, "output/output.avi");
    if (cfg.save_video) {
        try { std::filesystem::create_directories("output"); } catch (...) {}
    }
    cv::VideoWriter writer;
    bool writer_ok = false;

    // 主循环: 一帧一帧处理
    cv::Mat frame;
    int frame_count = 0;
    auto last_t = std::chrono::steady_clock::now();

    for (;;) {
        if (!input.read(frame)) break;
        ++frame_count;

        // 算一下处理速度(FPS)
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_t).count();
        last_t = now;
        double fps = (dt > 1e-6) ? (1.0 / dt) : 0.0;

        // 检测装甲板, 选目标, 解算位姿
        std::vector<Armor> armors = detector.detect(frame);
        const Armor* target = selector.select(armors, frame.cols, frame.rows);
        TargetState state = selector.state();

        Pose pose;
        VisionData data;
        data.state = state;
        if (target != nullptr) {
            pose = solver.solve(*target);
            if (pose.valid) {
                data.yaw_deg     = static_cast<float>(pose.yaw_deg);
                data.pitch_deg   = static_cast<float>(pose.pitch_deg);
                data.distance_mm = static_cast<float>(pose.distance_mm);
            } else {
                data.state = TargetState::NO_TARGET; // 解算失败按没目标处理
            }
        }

        // 打包成 8 字节报文, 模拟发送再解析回来, 打印并记录
        auto pkt = packer.pack(data);
        VisionData recv;
        bool checksum_ok = parser.parse(pkt, recv);
        std::string send = packer.formatSend(pkt);
        std::string rcv = parser.formatReceive(pkt, checksum_ok, recv);
        std::cout << send << std::endl;
        std::cout << rcv << std::endl;
        comm_log << send << std::endl;
        comm_log << rcv << std::endl;

        // 在画面上画框、写信息
        drawer.drawFrame(frame, armors, target, pose, fps, state);

        // 保存结果视频
        if (!writer_ok && cfg.save_video) {
            writer_ok = writer.open(out_path,
                                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                    input.fps(),
                                    cv::Size(frame.cols, frame.rows));
            if (!writer_ok)
                std::cerr << "结果视频创建失败: " << out_path << std::endl;
        }
        if (writer_ok) writer.write(frame);

        if (cfg.show_debug) {
            cv::imshow("ArmorAim", frame);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27) break;
        }
    }

    if (writer_ok) writer.release();
    std::cout << "处理完成, 一共 " << frame_count << " 帧" << std::endl;
    return 0;
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8); // Windows 控制台中文输出
#endif

    std::string config_path = findConfigPath(argc, argv);

    std::string video_override;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--video" && i + 1 < argc) {
            video_override = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
    }

    AppConfig cfg;
    ConfigLoader loader;
    if (!loader.load(config_path, cfg)) return 1;

    if (!video_override.empty()) {
        cfg.video_path = video_override;
        std::cout << "用命令行指定的视频: " << video_override << std::endl;
    }

    return runPipeline(cfg);
}
