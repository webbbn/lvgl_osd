cmake_minimum_required(VERSION 3.10)
project(lvgl_osd)
INCLUDE(FindPkgConfig)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member -O3 -DNDEBUG")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-address-of-packed-member -O3 -DNDEBUG")
if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_MONITOR -DUSE_MOUSE -DUSE_KEYBOARD")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG -DUSE_MONITOR -DUSE_MOUSE -DUSE_KEYBOARD")
endif (WIN32)

# Add the local cmake directory to the modules path
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/mavlink/standard")

file(GLOB_RECURSE INCLUDES "lvgl/*.h" "./*.h" )
file(GLOB_RECURSE SOURCES  "lvgl/*.c" )
if (WIN32)
  list(APPEND INCLUDES lv_drivers/win_drv.h)
  list(APPEND SOURCES lv_drivers/display/monitor.c lv_drivers/indev/mouse.c lv_drivers/indev/keyboard.c lv_drivers/win_drv.c)
  set(EXTRA_LIBS SDL2 mingw32 SDL2main ws2_32 SDL2)
else (WIN32)
  list(APPEND INCLUDES lv_drivers/display/fbdev.h)
  list(APPEND SOURCES lv_drivers/display/fbdev.c)
  set(EXTRA_LIBS pthread)
endif (WIN32)
pkg_search_module(SDL2 REQUIRED sdl2)
include_directories(${SDL2_INCLUDE_DIRS})
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
  sig_0.c
  sig_1.c
  sig_2.c
  sig_3.c
  sig_4.c
  sig_5.c
  sig_6.c
  north_needle.c
  compass.c
  ${SOURCES} ${INCLUDES})
target_link_libraries(lvgl_osd PRIVATE ${EXTRA_LIBS})
