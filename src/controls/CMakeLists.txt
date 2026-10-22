# controlsd. 라이브러리가 둘이다: 횡제어기·크루즈·출발 알림(control_core)은 플래너와 학습기 아래에 있고,
# 그 둘을 한 틱으로 묶는 ControlsTick(controls)은 위에 있다.
add_edgepilot_library(control_core
  adaptive_cruise.cc
  angle_fault_guard.cc
  control_holds.cc
  control_params.cc
  departure_alert.cc
  lateral_controller.cc
  lateral_nn.cc
  lateral_nn_select.cc
  lateral_torque.cc
)
target_link_libraries(control_core PUBLIC car utils_json)

# controlsd 한 틱의 로직(ControlsTick). 공유 메모리 없이 돌아 호스트 검사와 재생 도구가 같은 코드를 쓴다.
add_edgepilot_library(controls controls_tick.cc)
target_link_libraries(controls PUBLIC planning learners)

if(EDGEPILOT_BUILD_RUNTIME)
  add_executable(controlsd controlsd.cc)
  configure_board_target(controlsd common control_core planning controls)
endif()
