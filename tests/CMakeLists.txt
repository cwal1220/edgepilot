# 호스트 단위 테스트. host_tests가 전부를 빌드하고 ctest가 모은다(scripts/run_host_tests.sh).
include(FetchContent)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz
  URL_HASH SHA256=6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# add_host_test(<이름> <라이브러리...>): <이름>.cc 하나로 된 테스트. 저장소 루트에서 돈다
# (params/ 같은 상대 경로를 읽는 테스트가 있다).
add_custom_target(host_tests)
function(add_host_test target)
  add_host_tool(${target} ${target}.cc)
  target_link_libraries(${target} PRIVATE ${ARGN} GTest::gtest_main)
  gtest_discover_tests(${target} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  add_dependencies(host_tests ${target})
endfunction()

add_host_test(gtest_adaptive_cruise control_core)
add_host_test(gtest_alert_tones alert_tones)
add_host_test(gtest_background_writer)
add_host_test(gtest_calibration model)
add_host_test(gtest_car car)
add_host_test(gtest_control_holds control_core)
add_host_test(gtest_controls_tick controls)
add_host_test(gtest_departure_alert control_core)
add_host_test(gtest_desire_helper planning)
add_host_test(gtest_device_settings common utils_json)
add_host_test(gtest_hud_canvas hud)
add_host_test(gtest_hud_policy hud)
add_host_test(gtest_hud_state hud_state)
add_host_test(gtest_ipc_channels common)
add_host_test(gtest_lateral_controller control_core)
add_host_test(gtest_lateral_learners learners)
add_host_test(gtest_lateral_mpc planning)
add_host_test(gtest_lateral_planner planning)
add_host_test(gtest_lateral_torque control_core)
add_host_test(gtest_localization localization)
add_host_test(gtest_model_output common)
add_host_test(gtest_model_state_fill model hud_state)
add_host_test(gtest_panda_can_codec panda)
add_host_test(gtest_panda_firmware panda)
add_host_test(gtest_projection hud_state)
add_host_test(gtest_recording recording)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME check_web_console.py
         COMMAND ${Python3_EXECUTABLE} tests/check_web_console.py
         WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_test(NAME check_recording_reader.py
         COMMAND ${Python3_EXECUTABLE} tests/check_recording_reader.py
         WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
# numpy가 없으면 77로 끝난다(건너뜀)
set_tests_properties(check_recording_reader.py PROPERTIES SKIP_RETURN_CODE 77)
