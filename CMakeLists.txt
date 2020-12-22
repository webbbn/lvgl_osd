cmake_minimum_required(VERSION 3.10)
INCLUDE(FindPkgConfig)
project(lvgl)
set(CMAKE_C_STANDARD 11)#C11
set(CMAKE_CXX_STANDARD 17)#C17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/mavlink/standard")

file(GLOB_RECURSE INCLUDES "lv_drivers/*.h" "lv_examples/*.h"  "lvgl/*.h"  "./*.h" )
file(GLOB_RECURSE SOURCES  "lv_drivers/*.c" "lv_examples/*.c"  "lvgl/*.c" )

find_package(SDL2 REQUIRED SDL2)
include_directories(${SDL2_INCLUDE_DIRS})
if (WIN32)
  set(EXTRA_LIBS SDL2 mingw32 SDL2main ws2_32 SDL2)
else (WIN32)
  set(EXTRA_LIBS pthread ${SDL2_LIBRARIES})
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
  ${SOURCES} ${INCLUDES})
target_link_libraries(lvgl_osd PRIVATE ${EXTRA_LIBS})
