# 분석·재생·벤치 도구. 호스트 도구는 보드 라이브러리 없이 빌드된다.
add_host_tool(extract_lateral_dataset extract_lateral_dataset.cc)
target_link_libraries(extract_lateral_dataset PRIVATE control_core)

add_host_tool(replay_closed_loop replay_closed_loop.cc)
target_link_libraries(replay_closed_loop PRIVATE planning)

add_host_tool(replay_lateral_learners replay_lateral_learners.cc)
target_link_libraries(replay_lateral_learners PRIVATE learners)

add_host_tool(replay_planner replay_planner.cc)
target_link_libraries(replay_planner PRIVATE planning)

# HUD 스냅샷·타이밍 도구. 호스트와 보드에서 같은 렌더러로 그린다.
add_host_tool(hud_snapshot hud_snapshot.cc)
target_link_libraries(hud_snapshot PRIVATE hud)

# 알림음 미리 듣기(WAV). overlayd와 같은 합성 코드.
add_host_tool(alert_sound_preview alert_sound_preview.cc)
target_link_libraries(alert_sound_preview PRIVATE alert_tones)

add_host_tool(replay_localization replay_localization.cc)
target_link_libraries(replay_localization PRIVATE localization car)

# controlsd 한 틱의 로직(ControlsTick)을 녹화로 다시 돌린다. 리팩토링 전후 다이제스트 비교용.
add_host_tool(replay_controls replay_controls.cc)
target_link_libraries(replay_controls PRIVATE controls)

