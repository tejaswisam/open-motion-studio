cmake_minimum_required(VERSION 3.16)
project(oms_robot_model LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------

add_library(oms_robot_model
    src/robot_model.cpp
)

target_include_directories(oms_robot_model
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(oms_robot_model PRIVATE
    -Wall -Wextra -Wpedantic
)

# ---------------------------------------------------------------------------
# Optional: lightweight usage example (disable with -DOMS_BUILD_EXAMPLES=OFF)
# ---------------------------------------------------------------------------

option(OMS_BUILD_EXAMPLES "Build robot_model usage examples" ON)

if(OMS_BUILD_EXAMPLES)
    add_executable(robot_model_example examples/basic_robot.cpp)
    target_link_libraries(robot_model_example PRIVATE oms_robot_model)
endif()
