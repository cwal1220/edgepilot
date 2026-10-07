# panda USB 브리지: CAN 패킹과 펌웨어 이미지 검사는 호스트에서도 빌드하고, USB를 쓰는 pandad와
# panda_flash(런타임을 멈추고 펌웨어를 쓰는 도구)는 보드에서만.
add_edgepilot_library(panda panda_can_codec.cc panda_firmware.cc)

if(EDGEPILOT_BUILD_RUNTIME AND EDGEPILOT_BUILD_PANDA)
  add_executable(pandad
    pandad.cc
    panda_client.cc
    panda_flasher.cc
  )
  target_include_directories(pandad SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
  configure_board_target(pandad common panda ${LIBUSB_LIBRARIES})

  add_executable(panda_flash panda_flash.cc panda_flasher.cc)
  target_include_directories(panda_flash SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
  configure_board_target(panda_flash panda ${LIBUSB_LIBRARIES})
endif()
