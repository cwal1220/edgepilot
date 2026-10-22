// 재생 도구 연기 시험: RecordingWriter로 만든 짧은 합성 route를 도구마다 한 번 돌려, 끝까지 돌고 무엇인가 내는지
// 본다. 이벤트 파일이 아닌 경로를 주면 종료 코드 1이다(예전에는 'n=0 rmse=nan'을 찍고 성공으로 끝났다). 잘못된
// 옵션 값은 종료 코드 2다(cli_args.h). 도구는 이 시험과 같은 빌드의 bin 폴더에 있다(REPLAY_TOOL_DIR).
#include "cli_args.h"
#include "common/ipc_messages.h"
#include "recording/recording_format.h"
#include "recording/recording_writer.h"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

// 3초 동안 100 Hz 제어 상태와 CAN, 20 Hz 모델, 10 Hz IMU 묶음, 1 Hz 학습 상태를 쓴 route. 경로를 돌려준다.
std::string write_route(const std::string &root) {
    setenv("EDGEPILOT_RECORD_STAGING", (root + "/staging").c_str(), 1);
    const uint64_t t0 = 5'000'000'000ULL;
    {
        RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
        writer.set_enabled(true, t0);
        for (int tick = 0; tick < 300; ++tick) {
            const uint64_t t = t0 + static_cast<uint64_t>(tick) * 10'000'000ULL;
            CanBatch batch;
            batch.timestamp_ns = t;
            batch.valid = 1;
            batch.count = 1;
            batch.frames[0].address = 0x386;  // WHL_SPD11: 네 바퀴 60 km/h
            batch.frames[0].data_len = 8;
            for (int wheel = 0; wheel < 4; ++wheel) {
                const uint32_t raw = static_cast<uint32_t>(std::lround(60.0 / 0.03125));
                batch.frames[0].data[2 * wheel] = static_cast<uint8_t>(raw & 0xff);
                batch.frames[0].data[2 * wheel + 1] = static_cast<uint8_t>(raw >> 8);
            }
            writer.write_can(RecordType::CanRx, batch);
            ControlState control;
            control.timestamp_ns = t;
            control.active = control.engaged = control.vehicle_fresh = 1;
            control.ego_speed_kph = control.cluster_speed_kph = 60.0f;
            writer.write_state(RecordType::ControlState, t, &control, sizeof(control));
            if (tick % 5 == 0) {
                ModelState model;
                model.frame_id = static_cast<uint64_t>(tick / 5);
                model.capture_timestamp_ns = model.model_timestamp_ns = t;
                model.valid = 1;
                model.calibration.status = 1;
                for (int i = 0; i < kTrajectorySize; ++i) {
                    model.plan[i].x = 2.0f * static_cast<float>(i + 1);
                    model.lanes[1][i] = {-1.75f, 0.0f};
                    model.lanes[2][i] = {1.75f, 0.0f};
                }
                model.lane_probabilities[1] = model.lane_probabilities[2] = 0.9f;
                model.pose.valid = 1;
                model.pose.trans[0] = 16.7f;
                for (int k = 0; k < 3; ++k) model.pose.trans_std[k] = model.pose.rot_std[k] = 0.1f;
                writer.write_state(RecordType::ModelState, t, &model, sizeof(model));
            }
            if (tick % 10 == 0) {
                ImuBatch imu;
                imu.count = 10;
                for (unsigned s = 0; s < imu.count; ++s) {
                    imu.samples[s].timestamp_ns = t + s * 10'000'000ULL;
                    imu.samples[s].accel_mps2[1] = 9.81f;
                }
                writer.write_state(RecordType::Imu, t, &imu, sizeof(imu));
            }
            if (tick % 100 == 0) {
                LearnerState learner;
                learner.timestamp_ns = t;
                learner.steer_ratio = 15.0f;
                learner.lat_accel_factor = 2.75f;
                writer.write_state(RecordType::LearnerState, t, &learner, sizeof(learner));
            }
        }
        writer.set_enabled(false, t0 + 4'000'000'000ULL);
        writer.close();
    }
    for (const auto &entry : std::filesystem::directory_iterator(root + "/recordings")) return entry.path().string();
    return {};
}

// 도구를 돌려 종료 코드와 표준 출력을 돌려준다.
int run_tool(const std::string &command, std::string *output = nullptr) {
    const std::string log = std::filesystem::temp_directory_path() / ("replay_tool_" + std::to_string(getpid()));
    const int status = std::system((std::string(REPLAY_TOOL_DIR) + "/" + command + " > " + log + " 2>&1").c_str());
    if (output) {
        std::ifstream in(log);
        std::stringstream text;
        text << in.rdbuf();
        *output = text.str();
    }
    std::filesystem::remove(log);
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

size_t line_count(const std::string &path) {
    std::ifstream in(path);
    size_t lines = 0;
    for (std::string line; std::getline(in, line);) ++lines;
    return lines;
}

TEST(ReplayTools, RunOnASyntheticRoute) {
    char root_template[] = "/tmp/gtest_replay_tools_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string route = write_route(root);
    ASSERT_FALSE(route.empty());
    const std::string events = route + "/events/000.bin";
    ASSERT_TRUE(std::filesystem::exists(events));
    const std::string missing = root + "/missing.bin";
    std::string out;

    EXPECT_EQ(run_tool("replay_controls " + events, &out), 0) << out;
    EXPECT_NE(out.find("ticks 299 "), std::string::npos) << "3초 동안 10 ms 틱(첫 기록 다음부터)\n" << out;
    EXPECT_EQ(run_tool("replay_controls " + events + " " + missing, &out), 1) << "못 연 파일";

    EXPECT_EQ(run_tool("replay_planner " + root + "/planner.csv " + events, &out), 0) << out;
    EXPECT_GT(line_count(root + "/planner.csv"), 50u) << "모델 프레임마다 한 줄";
    EXPECT_EQ(run_tool("replay_planner " + root + "/planner.csv " + missing), 1);

    EXPECT_EQ(run_tool("replay_closed_loop " + root + "/closed.csv " + events, &out), 0) << out;
    EXPECT_EQ(run_tool("replay_closed_loop - " + missing, &out), 1) << out;

    EXPECT_EQ(run_tool("replay_localization " + root + "/loc.csv " + events, &out), 0) << out;
    EXPECT_GT(line_count(root + "/loc.csv"), 10u) << "IMU 묶음마다 한 줄";
    EXPECT_EQ(run_tool("replay_localization " + root + "/loc.csv " + missing), 1);

    EXPECT_EQ(run_tool("replay_lateral_learners " + events, &out), 0) << out;
    EXPECT_EQ(run_tool("replay_lateral_learners " + missing), 1);

    EXPECT_EQ(run_tool("extract_lateral_dataset " + root + "/dataset.csv " + events, &out), 0) << out;
    EXPECT_GT(line_count(root + "/dataset.csv"), 250u) << "ControlState마다 한 줄";
    EXPECT_EQ(run_tool("extract_lateral_dataset " + root + "/dataset.csv " + missing), 1);

    // 숫자가 아니거나 범위 밖인 값, 값이 없는 옵션은 기본값으로 돌지 않고 사용법 오류로 끝난다
    for (const std::string &args :
         {"replay_planner --lane-filter abc " + root + "/planner.csv " + events,
          "replay_controls --lane-filter 1.5 " + events, "replay_controls " + events + " --steering",
          "replay_closed_loop --wn 16x " + root + "/closed.csv " + events,
          "replay_closed_loop --gain-pts 1,2,3 " + root + "/closed.csv " + events,
          "replay_closed_loop --delay -1 " + root + "/closed.csv " + events,
          "replay_lateral_learners --metric-from 1e3 " + events,
          "replay_planner " + root + "/planner.csv " + events + " --steering",
          "replay_localization --lag " + root + "/loc.csv " + events}) {
        EXPECT_EQ(run_tool(args, &out), 2) << args << "\n" << out;
    }

    std::filesystem::remove_all(root);
}

// 옵션은 어디에 와도 되고, 값이 없거나 틀리면 error()다. 위치 인자는 순서대로 남는다.
TEST(CliArgs, ArgReaderReadsOptionsAndPositionals) {
    const char *argv_ok[] = {"tool", "a.csv", "--gain", "0.5", "--open", "b.bin", "--name", "x"};
    ArgReader ok(8, const_cast<char **>(argv_ok));
    double gain = 0.0;
    bool open = false;
    std::string name;
    std::vector<std::string> positional;
    while (ok.next()) {
        if (ok.is("--gain"))
            ok.read(&gain, 0.0, 1.0);
        else if (ok.is("--open"))
            open = true;
        else if (ok.is("--name"))
            name = ok.value();
        else if (ok.is_option())
            ok.fail();
        else
            positional.push_back(ok.arg());
    }
    EXPECT_FALSE(ok.error());
    EXPECT_DOUBLE_EQ(gain, 0.5);
    EXPECT_TRUE(open);
    EXPECT_EQ(name, "x");
    EXPECT_EQ(positional, (std::vector<std::string>{"a.csv", "b.bin"}));

    const char *argv_missing[] = {"tool", "--name"};
    ArgReader missing(2, const_cast<char **>(argv_missing));
    while (missing.next())
        if (missing.is("--name")) EXPECT_STREQ(missing.value(), "");
    EXPECT_TRUE(missing.error()) << "값이 없는 옵션";

    const char *argv_bad[] = {"tool", "--gain", "2"};
    ArgReader bad(3, const_cast<char **>(argv_bad));
    double kept = 0.25;
    while (bad.next())
        if (bad.is("--gain")) bad.read(&kept, 0.0, 1.0);
    EXPECT_TRUE(bad.error()) << "범위 밖";
    EXPECT_DOUBLE_EQ(kept, 0.25) << "틀린 값은 담지 않는다";
}

TEST(CliArgs, WholeFiniteNumbersInRange) {
    double d = 0.0;
    EXPECT_TRUE(parse_double_arg("0.42", &d, 0.0, 1.0));
    EXPECT_DOUBLE_EQ(d, 0.42);
    for (const char *bad : {"", "abc", "0.3s", "1.5", "nan", "inf", "1e999"}) {
        d = -1.0;
        EXPECT_FALSE(parse_double_arg(bad, &d, 0.0, 1.0)) << bad;
        EXPECT_EQ(d, -1.0) << "실패하면 값을 바꾸지 않는다: " << bad;
    }
    EXPECT_FALSE(parse_double_arg(nullptr, &d));

    int n = 0;
    EXPECT_TRUE(parse_int_arg("-3", &n));
    EXPECT_EQ(n, -3);
    EXPECT_FALSE(parse_int_arg("3.0", &n));
    EXPECT_FALSE(parse_int_arg("0", &n, 1)) << "하한";
    EXPECT_FALSE(parse_int_arg("99999999999", &n)) << "int 범위 밖";
    long l = 0;
    EXPECT_FALSE(parse_long_arg("12 ", &l)) << "뒤에 남은 글자";

    float pts[4] = {9.0f, 9.0f, 9.0f, 9.0f};
    EXPECT_TRUE(parse_float_list_arg("1.05,0.385,1.04,1.74", pts, 4));
    EXPECT_FLOAT_EQ(pts[1], 0.385f);
    for (const char *bad : {"1,2,3", "1,2,3,4,5", "1,,3,4", "1,2,3,4,", "1;2;3;4", "1,2,x,4"}) {
        float keep[4] = {9.0f, 9.0f, 9.0f, 9.0f};
        EXPECT_FALSE(parse_float_list_arg(bad, keep, 4)) << bad;
        EXPECT_EQ(keep[0], 9.0f) << "실패하면 값을 바꾸지 않는다: " << bad;
    }
}

}  // namespace
