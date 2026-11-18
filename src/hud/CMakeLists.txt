# HUD. 공유 상태 → 표시 상태(hud_state), 그리기와 화면 밖 판단(hud), 알림음 합성(alert_tones)은
# 자체 캔버스·구운 글꼴만 써서 호스트에서도 빌드되고, 검사와 hud_snapshot이 보드와 같은 그림을 그린다.
# 알림음 재생(alert_sound)과 overlayd는 보드에서만.
add_edgepilot_library(hud_state hud_state.cc)
target_link_libraries(hud_state PUBLIC common)

add_edgepilot_library(hud
  hud_canvas.cc
  hud_cards.cc
  hud_font.cc
  hud_policy.cc
  hud_renderer.cc
  hud_scene.cc
  recording_switch.cc
)
target_link_libraries(hud PUBLIC hud_state utils_json)

# 알림음 합성. 호스트에서 alert_sound_preview가 같은 소리를 낸다.
add_edgepilot_library(alert_tones alert_tones.cc)

if(EDGEPILOT_BUILD_RUNTIME)
  add_edgepilot_library(alert_sound alert_sound.cc)
  target_link_libraries(alert_sound PUBLIC alert_tones)

  add_executable(overlayd
    overlayd.cc
    system_monitor.cc
  )
  configure_board_target(overlayd common hud alert_sound maixcam2_media utils_json)
endif()
