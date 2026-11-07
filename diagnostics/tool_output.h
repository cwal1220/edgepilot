#pragma once

// 재생 도구의 출력 경로 검사. 출력이 첫 인자인 도구에서 그 인자를 빠뜨리면 녹화의 events/000.bin이 출력 자리에 들어가
// 잘려 나갔다(2026-10-17, 10-21: 두 route가 첫 1분을 잃었다). 녹화 파일(이벤트 로그·프레임 색인)이거나 route 폴더
// 안(<route>/x, <route>/events/x, <route>/segments/000/x)의 경로면 쓰지 않는다. 도구는 false면 종료 코드 2로 끝낸다.

#include "recording/recording_format.h"  // kEventLogMagic, kFrameIndexMagic

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

// events/가 있거나 manifest.json이 있는 폴더(manifest 없이 옮긴 옛 route도 있다)
inline bool is_route_dir(const std::filesystem::path &dir) {
    std::error_code ec;
    return std::filesystem::is_directory(dir / "events", ec) || std::filesystem::exists(dir / "manifest.json", ec);
}

inline bool is_recording_file(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    char magic[sizeof(kEventLogMagic)] = {};
    if (!in.read(magic, sizeof(magic))) return false;
    return std::memcmp(magic, kEventLogMagic, sizeof(magic)) == 0 ||
           std::memcmp(magic, kFrameIndexMagic, sizeof(magic)) == 0;
}

inline bool tool_output_allowed(const std::string &path) {
    std::error_code ec;
    const std::filesystem::path out = std::filesystem::absolute(path, ec);
    bool refused = is_recording_file(out);
    std::filesystem::path dir = out.parent_path();
    for (int level = 0; level < 3 && !refused && !dir.empty(); ++level, dir = dir.parent_path()) {
        refused = is_route_dir(dir);
    }
    if (refused) {
        std::fprintf(stderr, "refusing to write %s: it is a recording or inside a route (output argument missing?)\n",
                     path.c_str());
    }
    return !refused;
}
