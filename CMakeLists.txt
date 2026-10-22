cmake_minimum_required(VERSION 3.16)

project(edgepilot LANGUAGES C CXX)

option(EDGEPILOT_BUILD_DIAGNOSTICS "Build the host unit tests and diagnostic tools" OFF)
option(EDGEPILOT_BUILD_PANDA "Build optional panda USB/CAN bridge" ON)
option(EDGEPILOT_BUILD_RUNTIME "Build AX630C runtime binaries" ON)
option(EDGEPILOT_WERROR "Treat compiler warnings in edgepilot code as errors" OFF)
option(EDGEPILOT_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer (host tests)" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(config_type DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_type} "${CMAKE_BINARY_DIR}/bin")
endforeach()

# MaixCAM2(AX630C) 보드 이미지의 NPU 런타임. 엔진 헤더는 MSP SDK에 없어 src/model/ax_engine_api.h로 선언한다.
set(AX_LIB_DIR "/opt/lib" CACHE PATH "AX630C runtime library directory to link against")
set(AX_RPATH "/opt/lib" CACHE PATH "where the board keeps libax_engine/libax_sys at run time")

set(PROJECT_WARNINGS -Wall -Wextra)
# 경고가 없는 지금 상태를 고정할 때(-DEDGEPILOT_WERROR=ON). 외부 코드(SDK·googletest)는 이 목록을 쓰지 않는다.
if(EDGEPILOT_WERROR)
  list(APPEND PROJECT_WARNINGS -Werror)
endif()
# 메모리·정의되지 않은 동작 검사(-DEDGEPILOT_SANITIZE=ON). 호스트 시험을 이것으로 돌린다: 보드의 SDK 라이브러리는
# 계측되지 않는다.
if(EDGEPILOT_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
  add_link_options(-fsanitize=address,undefined)
endif()
set(PROJECT_LOCAL_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# 라이브러리: 로컬 헤더(src/ 기준 경로로 부른다)와 경고. 폴더마다 src/<폴더>/CMakeLists.txt가 정의한다.
function(add_edgepilot_library target)
  add_library(${target} STATIC ${ARGN})
  target_include_directories(${target} PUBLIC ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()

# 호스트 검사·도구: 로컬 헤더와 경고만 붙인 일반 실행 파일.
function(add_host_tool target)
  add_executable(${target} ${ARGN})
  target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()

if(EDGEPILOT_BUILD_RUNTIME)
  find_package(PkgConfig QUIET)
  if(EDGEPILOT_BUILD_PANDA)
    if(PkgConfig_FOUND)
      pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    else()
      message(FATAL_ERROR "PkgConfig is required when EDGEPILOT_BUILD_PANDA=ON")
    endif()
  endif()

  find_library(AX_ENGINE_LIB ax_engine PATHS "${AX_LIB_DIR}" NO_DEFAULT_PATH)
  find_library(AX_SYS_LIB ax_sys PATHS "${AX_LIB_DIR}" NO_DEFAULT_PATH)
  if(NOT AX_ENGINE_LIB OR NOT AX_SYS_LIB)
    message(FATAL_ERROR "libax_engine/libax_sys not found in ${AX_LIB_DIR} (set AX_LIB_DIR)")
  endif()

  # MaixCAM2 카메라(VI)·화면(VO). scripts/fetch_maixcam2_sdk.sh가 받은 MSP SDK와
  # MaixCDK 헤더로 보드의 libmaixcam_lib(ax_middleware)를 직접 쓴다.
  set(MAIX_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/deps/ax630")
  set(MAIX_MSP_OVERRIDE "${MAIX_DEPS}/MaixCDK/components/3rd_party/maixcam2_msp/msp/sample/common")
  if(NOT EXISTS "${MAIX_DEPS}/MaixCDK/components/maixcam_lib/include/maixcam2/ax_middleware.hpp")
    message(FATAL_ERROR "MaixCAM2 SDK missing: run scripts/fetch_maixcam2_sdk.sh")
  endif()
  file(GLOB_RECURSE _msp_component_headers "${MAIX_DEPS}/maixcam2_msp/component/*/include/*.h")
  set(MAIX_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2/stub"
    "${MAIX_DEPS}/MaixCDK/components/maixcam_lib/include/maixcam2"
    "${MAIX_DEPS}/MaixCDK/components/basic/include"
    "${MAIX_DEPS}/MaixCDK/components/vision/include"
    "${MAIX_MSP_OVERRIDE}"
    "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include"
    "${MAIX_DEPS}/maixcam2_msp/sample/common")
  foreach(_h IN LISTS _msp_component_headers)
    get_filename_component(_d "${_h}" DIRECTORY)
    list(APPEND MAIX_INCLUDES "${_d}")
  endforeach()
  list(REMOVE_DUPLICATES MAIX_INCLUDES)
  file(GLOB AX_ALL_LIBS "${AX_LIB_DIR}/libax_*.so")

  # 보드 SDK 래퍼(maixcam2_media·gdc·venc·vdec). 헤더는 platform/ 기준 경로로 부른다
  # (#include "maixcam2/maix_display.h").
  add_subdirectory(platform/maixcam2)

  # 보드 실행 파일: 로컬 헤더, 경고, 빌드 버전(common/build_version.h), 공용 시스템 라이브러리. 폴더마다 자기
  # 프로세스를 정의한다.
  function(configure_board_target target)
    target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
    target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
    target_link_options(${target} PRIVATE "-Wl,-rpath,${AX_RPATH}" "-Wl,-rpath-link,${AX_LIB_DIR}")
    target_link_libraries(${target} PRIVATE ${ARGN} build_version pthread dl rt)
  endfunction()
endif()

# 런타임 코드(src/<하위 시스템>/): 호스트에서도 빌드되는 라이브러리와, 보드에서만 빌드되는 프로세스.
add_subdirectory(src)

if(EDGEPILOT_BUILD_DIAGNOSTICS)
  # 단위 테스트는 호스트 빌드(런타임 끔)에서만 빌드·실행한다. 보드 빌드는 도커 안의 arm64 네이티브 빌드라
  # 교차 빌드로 보이지 않아, 런타임을 켰는지로 가른다. enable_testing()은 루트에 있어야 ctest가 하위 폴더를 찾는다.
  if(NOT EDGEPILOT_BUILD_RUNTIME)
    enable_testing()
    add_subdirectory(tests)
  endif()
  add_subdirectory(diagnostics)
endif()
