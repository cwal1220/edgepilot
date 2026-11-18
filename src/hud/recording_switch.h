#pragma once

// HUD에서 녹화를 켜고 끄기: params/recording.json의 enabled를 뒤집는다. 웹 콘솔의 녹화 스위치와 같은 파일이고,
// recordd가 그 파일을 보고 녹화를 시작하거나 닫는다(같은 임시 파일 + rename으로 쓴다).

#include <string>

// path의 enabled를 뒤집어 쓴다. 다른 키와 파일 모양(들여쓰기, 키 순서)은 그대로다: enabled 값 글자만 바꾸고, 바꾼
// 글을 다시 풀어 enabled가 반대 값인지 본 뒤 쓴다. 파일이 없거나 JSON 객체가 아니거나 enabled가 불리언이 아니면,
// 또는 쓰지 못하면 파일을 그대로 두고 false와 error다. 성공하면 enabled에 새 값.
bool toggle_recording_enabled(const std::string &path, bool *enabled, std::string *error);
