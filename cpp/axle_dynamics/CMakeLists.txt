cmake_minimum_required(VERSION 3.20)
project(axle_dynamics_native LANGUAGES CXX)

add_library(axle_dynamics_native SHARED axle_kernel.cpp)
target_compile_features(axle_dynamics_native PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(axle_dynamics_native PRIVATE /W4 /WX /O2 /fp:precise)
else()
    target_compile_options(
        axle_dynamics_native PRIVATE
        -Wall -Wextra -Werror -O2 -fno-fast-math
    )
endif()

set_target_properties(
    axle_dynamics_native PROPERTIES
    OUTPUT_NAME axle_dynamics_native
    RUNTIME_OUTPUT_DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/suspension_multibody/src/suspension_multibody/native"
    LIBRARY_OUTPUT_DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/suspension_multibody/src/suspension_multibody/native"
)
