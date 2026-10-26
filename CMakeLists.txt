cmake_minimum_required(VERSION 3.20)
project(dm_joint_lab C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- module 层：与 MCU 无关，主机可编可测 ----
add_library(dm_module STATIC
  module/dm_motor/dm_motor.c
)
target_include_directories(dm_module PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/module/dm_motor
  ${CMAKE_CURRENT_SOURCE_DIR}/platform
)
target_compile_options(dm_module PRIVATE -Wall -Wextra -Werror)

find_library(M_LIB m)
if(M_LIB)
  target_link_libraries(dm_module PUBLIC ${M_LIB})
endif()

# ---- 主机侧测试（ASan/UBSan）----
option(DMJOINT_HOST_TESTS "Build host-side spec tests" ON)
if(DMJOINT_HOST_TESTS)
  add_executable(host_tests tests/host/main.c)
  target_link_libraries(host_tests PRIVATE dm_module)
  target_compile_options(host_tests PRIVATE -Wall -Wextra -Werror
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(host_tests PRIVATE -fsanitize=address,undefined)
endif()
