# 횡 플래너: 차선 변경 desire, 횡 MPC, 경로 선택.
add_edgepilot_library(planning
  desire_helper.cc
  lateral_mpc.cc
  lateral_planner.cc
)
target_link_libraries(planning PUBLIC control_core common)
