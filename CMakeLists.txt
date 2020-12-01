cmake_minimum_required(VERSION 3.10)
project(lvgl_osd)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member -O3 -DNDEBUG")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-address-of-packed-member -O3 -DNDEBUG")

# Add the local cmake directory to the modules path
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/mavlink/standard")

file(GLOB_RECURSE INCLUDES "lv_drivers/*.h" "lvgl/*.h" "./*.h" )
file(GLOB_RECURSE SOURCES  "lv_drivers/*.c"  "lvgl/*.c" )

find_package(SDL2 REQUIRED SDL2)
include_directories(${SDL2_INCLUDE_DIRS})
add_executable(lvgl_osd lvgl_osd.cc telemetry.cc attitude_background.c img_hand.c ${SOURCES} ${INCLUDES})
target_link_libraries(lvgl_osd PRIVATE SDL2 pthread)
