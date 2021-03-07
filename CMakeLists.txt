cmake_minimum_required(VERSION 3.10)
INCLUDE(FindPkgConfig)
project(lvgl)
set(CMAKE_C_STANDARD 11)#C11
set(CMAKE_CXX_STANDARD 17)#C17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

option(USE_MPV "Add support for playing a video URL in the background using MPV" OFF)
option(USE_FFMPEG "Add support for playing a video URL in the background using FFMPEG" OFF)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/mavlink/standard")

file(GLOB_RECURSE INCLUDES "lv_drivers/*.h" "lv_examples/*.h"  "lvgl/*.h"  "./*.h" )
file(GLOB_RECURSE SOURCES  "lv_drivers/*.c" "lv_examples/*.c"  "lvgl/*.c" )

if (${USE_FFMPEG})
  message("FFMPEG video support")
  pkg_check_modules(MODULES REQUIRED
    libavcodec
    libavformat
    libavutil
    libva
    gl
    egl
    libdrm
    freetype2)
  include_directories(${MODULES_INCLUDE_DIRS})
  set(EXTRA_LIBS ${MODULES_LIBRARIES} X11 va-x11 va-drm pthread)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_FFMPEG_MONITOR")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_FFMPEG_MONITOR")
else ()
  find_package(SDL2 REQUIRED SDL2)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_MPV_MONITOR")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_MPV_MONITOR")
  include_directories(${SDL2_INCLUDE_DIR} ${MPV_INCLUDE_DIR} ${MODULES_INCLUDE_DIRS})
  if (${USE_MPV})
    message("MPV video support")
    pkg_check_modules(MPV REQUIRED mpv)
    include_directories(${MPV_INCLUDE_DIR})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_MPV -DUSE_MPV_MONITOR")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_MPV -DUSE_MPV_MONITOR")
  else()
    message("NO video support")
  endif ()
endif ()

if (WIN32)
  set(EXTRA_LIBS ${MPV_LIBRARIES} SDL2 mingw32 SDL2main ws2_32 SDL2 opengl32)
else (WIN32)
  set(EXTRA_LIBS ${EXTRA_LIBS} pthread)
endif (WIN32)

add_executable(lvgl_osd
  lvgl_osd.cc
  telemetry.cc
  attitude_background.c
  attitude_foreground.c
  attitude_ground.c
  attitude_ring.c
  home_arrow_ring.c
  north_arrow_ring.c
  satellite.c
  bat_full.c
  bat_0.c
  bat_1.c
  bat_2.c
  bat_3.c
  bat_4.c
  sig_0.c
  sig_1.c
  sig_2.c
  sig_3.c
  sig_4.c
  sig_5.c
  sig_6.c
  compass.c
  home_arrow.c
  egl_video.cc
  ffmpeg_decoder.cc
  ffmpeg_monitor.cc
  mpv_monitor.c
  ${SOURCES} ${INCLUDES})
target_link_libraries(lvgl_osd PRIVATE ${EXTRA_LIBS} ${SDL2_LIBRARIES} ${MPV_LIBRARIES})
