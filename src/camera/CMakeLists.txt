# camerad. 보드에서만 빌드한다.
if(EDGEPILOT_BUILD_RUNTIME)
  add_executable(camerad camerad.cc)
  configure_board_target(camerad common maixcam2_media)
endif()
