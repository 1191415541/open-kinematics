#> Build the suspension kernel shared library with CMake + Ninja.
#>
#> This is the real build.  `packages/suspension_multibody/scripts/build_axle_native.py`
#> is a compatibility wrapper that delegates here and then copies the product into
#> the axle-semantics package, where the ctypes boundary loads it from.
#>
#> The source list is explicit on purpose: CMake does not glob, and a missing
#> entry shows up only as an undefined symbol at load time.  K2 split the former
#> single translation unit into one file per module; K3 onwards moves those
#> modules into layered libraries.
cmake_minimum_required(VERSION 3.20)
project(suspension_kernel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    message(
        FATAL_ERROR
        "CMAKE_BUILD_TYPE must be set explicitly: the flag set differs per "
        "configuration and silently defaulting changes the product."
    )
endif()

set(KERNEL_SOURCES
    cpp/src/abi/kernel_abi.cpp
    cpp/src/abi/kernel_core.cpp
)

#: One library per module (K6).  The tire models have their own targets above; the
#: rest are created here from the module directories.  `src/abi` stays in the shared
#: library: the ABI layer *is* the aggregate, so putting it in an archive would only
#: add a link-order dependency on everything below it.

set(MB_SOLVE_DYNAMIC_SOURCES
    cpp/src/solve_dynamic/kernel_events.cpp
    cpp/src/solve_dynamic/kernel_integrator.cpp
    cpp/src/solve_dynamic/kernel_integrator_newton.cpp
    cpp/src/solve_dynamic/kernel_integrator_residual.cpp
    cpp/src/solve_dynamic/kernel_integrator_step.cpp
    cpp/src/solve_dynamic/kernel_integrator_state.cpp
)
set(MB_INPUT_SOURCES
    cpp/src/input/kernel_input.cpp
)
set(MB_SOLVE_STATIC_SOURCES
    cpp/src/solve_static/kernel_static_contact.cpp
    cpp/src/solve_static/kernel_static_projection.cpp
    cpp/src/solve_static/kernel_static_trim.cpp
)
set(MB_CONTRACT_SOURCES
    cpp/src/contract/contract_container.cpp
    cpp/src/contract/contract_json.cpp
    cpp/src/contract/contract_registry.cpp
    cpp/src/contract/contract_sha256.cpp
)
#: The case layer translates contract documents into solver runs.  It sits at
#: L5: it may read the contract and describe the model, but it never calls the
#: solver, so `abi` stays the only module that knows the call sequence.
set(MB_CASES_SOURCES
    cpp/src/cases/axle_dynamic.cpp
    cpp/src/cases/case_dispatch.cpp
    cpp/src/cases/contract_model.cpp
    cpp/src/cases/handling.cpp
    cpp/src/cases/kc_quasi_static.cpp
    cpp/src/cases/ride_four_post.cpp
    cpp/src/cases/ride_random_road.cpp
    cpp/src/cases/vehicle_dynamic.cpp
    cpp/src/cases/vehicle_kc.cpp
)
set(MB_OUTPUT_SOURCES
    cpp/src/output/kernel_output.cpp
)
set(MB_ASSEMBLY_SOURCES
    cpp/src/assembly/build_model.cpp
    cpp/src/assembly/element_reader.cpp
    cpp/src/assembly/registration.cpp
)
set(MB_FORCE_SOURCES
    cpp/src/force/directional.cpp
    cpp/src/force/external_vector.cpp
    cpp/src/force/layout.cpp
)
set(MB_ELEMENT_SOURCES
    cpp/src/element/anti_roll.cpp
    cpp/src/element/bushing.cpp
    cpp/src/element/curves.cpp
    cpp/src/element/directional.cpp
    cpp/src/element/drive_brake.cpp
    cpp/src/element/spring.cpp
    cpp/src/element/steering.cpp
)
set(MB_TIRE_SOURCES
    cpp/src/tire/assemble.cpp
    cpp/src/tire/directional.cpp
    cpp/src/tire/kernel_tire_assembly.cpp
)
set(MB_JOINT_SOURCES
    cpp/src/joint/kernel_directional_constraint.cpp
    cpp/src/joint/kernel_model_constraint.cpp
    cpp/src/joint/registry.cpp
)
set(MB_MODEL_SOURCES
    cpp/src/model/directional.cpp
    cpp/src/model/kernel_model.cpp
    cpp/src/model/kernel_model_accessors.cpp
    cpp/src/model/road.cpp
)
set(MB_TIRE_PAC2002_SOURCES
    cpp/src/tire/pac2002/contact_mass.cpp
    cpp/src/tire/pac2002/turn_slip.cpp
    cpp/src/tire/pac2002/directional.cpp
    cpp/src/tire/pac2002/parameters.cpp
    cpp/src/tire/pac2002/law.cpp
    cpp/src/tire/pac2002/scope.cpp
    cpp/src/tire/pac2002/model.cpp
    cpp/src/tire/pac2002/spin.cpp
    cpp/src/tire/pac2002/vertical.cpp
)
set(MB_TIRE_FIALA_SOURCES
    cpp/src/tire/fiala/relaxation.cpp
    cpp/src/tire/fiala/directional.cpp
    cpp/src/tire/fiala/forces.cpp
    cpp/src/tire/fiala/model.cpp
    cpp/src/tire/fiala/modes.cpp
)
set(MB_TIRE_BRUSH_SOURCES
    cpp/src/tire/brush/model.cpp
)
set(MB_TIRE_COMMON_SOURCES
    cpp/src/tire/common/kinematics.cpp
)
set(MB_TIRE_STATE_SOURCES
    cpp/src/tire_state/kernel_tire_state.cpp
)
set(MB_LINEAR_SOURCES
    cpp/src/linear/kernel_linear.cpp
)
set(MB_NUMERIC_SOURCES
    cpp/src/numeric/angles.cpp
    cpp/src/numeric/kernel_numeric.cpp
)
set(MB_DUAL_SOURCES
    cpp/src/dual/curves.cpp
    cpp/src/dual/dual_geometry.cpp
    cpp/src/dual/kernel_dual_algebra.cpp
)
set(MB_CONFIG_SOURCES
    cpp/src/config/kernel_config.cpp
)

set(MB_ABI_SOURCES
    cpp/src/abi/kernel_abi.cpp
    cpp/src/abi/kernel_contract_run.cpp
    cpp/src/abi/kernel_core.cpp
)

#: A shared set of compile options applied to every kernel target.  Defined once
#: so a per-layer library cannot drift from the aggregate: `-fopenmp` in
#: particular must be identical across all translation units, because without it
#: the `#pragma omp` directives are ignored while the runtime still reports a
#: thread count, which changes behaviour silently.
set(KERNEL_COMPILE_OPTIONS
    -Wall
    -Wextra
    -Werror
    -fno-fast-math
    -fopenmp
)

#: `mb_numeric`/`mb_dual`/`mb_config` (the split of `mb_base`, subtask 03) and
#: `mb_linear` (the rename of `mb_linalg`) are the libraries the epic requires to be free of any
#: `Model`/`ResidualContext` dependency.  Both source files were verified
#: to reference neither, so the boundary can be drawn by target membership alone
#: -- no code has to move for this step.  They are STATIC so the aggregate still
#: produces one self-contained shared library, and POSITION_INDEPENDENT_CODE is
#: required because they end up inside one.
add_library(mb_numeric STATIC ${MB_NUMERIC_SOURCES})
add_library(mb_dual STATIC ${MB_DUAL_SOURCES})
add_library(mb_config STATIC ${MB_CONFIG_SOURCES})
add_library(mb_linear STATIC ${MB_LINEAR_SOURCES})
set_target_properties(
    mb_numeric mb_dual mb_config mb_linear
    PROPERTIES POSITION_INDEPENDENT_CODE ON
)
target_compile_options(mb_numeric PRIVATE ${KERNEL_COMPILE_OPTIONS})
target_compile_options(mb_dual PRIVATE ${KERNEL_COMPILE_OPTIONS})
target_compile_options(mb_config PRIVATE ${KERNEL_COMPILE_OPTIONS})
target_compile_options(mb_linear PRIVATE ${KERNEL_COMPILE_OPTIONS})
target_link_options(mb_linear PRIVATE -fopenmp)


add_library(mb_tire_pac2002 STATIC ${MB_TIRE_PAC2002_SOURCES})
add_library(mb_solve_dynamic STATIC ${MB_SOLVE_DYNAMIC_SOURCES})
add_library(mb_input STATIC ${MB_INPUT_SOURCES})
add_library(mb_contract STATIC ${MB_CONTRACT_SOURCES})
add_library(mb_cases STATIC ${MB_CASES_SOURCES})
add_library(mb_solve_static STATIC ${MB_SOLVE_STATIC_SOURCES})
add_library(mb_output STATIC ${MB_OUTPUT_SOURCES})
add_library(mb_assembly STATIC ${MB_ASSEMBLY_SOURCES})
add_library(mb_force STATIC ${MB_FORCE_SOURCES})
add_library(mb_element STATIC ${MB_ELEMENT_SOURCES})
add_library(mb_tire STATIC ${MB_TIRE_SOURCES})
add_library(mb_joint STATIC ${MB_JOINT_SOURCES})
add_library(mb_model STATIC ${MB_MODEL_SOURCES})
add_library(mb_tire_common STATIC ${MB_TIRE_COMMON_SOURCES})
add_library(mb_tire_state STATIC ${MB_TIRE_STATE_SOURCES})
add_library(mb_tire_fiala STATIC ${MB_TIRE_FIALA_SOURCES})
add_library(mb_tire_brush STATIC ${MB_TIRE_BRUSH_SOURCES})
set_target_properties(
    mb_tire_pac2002 mb_tire_fiala mb_tire_brush
    PROPERTIES POSITION_INDEPENDENT_CODE ON
)
foreach(_tire_target mb_tire_pac2002 mb_tire_fiala mb_tire_brush)
    target_include_directories(
        ${_tire_target}
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/cpp/axle_dynamics"
        "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include"
    )
    target_compile_options(${_tire_target} PRIVATE ${KERNEL_COMPILE_OPTIONS})
    target_link_options(${_tire_target} PRIVATE -fopenmp)
endforeach()

foreach(_module_target mb_contract mb_cases mb_solve_dynamic mb_solve_static mb_input mb_output mb_assembly mb_force mb_element mb_tire mb_joint mb_model mb_tire_pac2002 mb_tire_fiala mb_tire_brush mb_tire_common mb_tire_state mb_linear mb_numeric mb_dual mb_config)
    set_target_properties(
        ${_module_target} PROPERTIES POSITION_INDEPENDENT_CODE ON
    )
    target_include_directories(
        ${_module_target}
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/cpp/axle_dynamics"
        "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include"
    )
    target_compile_options(${_module_target} PRIVATE ${KERNEL_COMPILE_OPTIONS})
    target_link_options(${_module_target} PRIVATE -fopenmp)
endforeach()

add_library(suspension_kernel SHARED ${MB_ABI_SOURCES})
target_include_directories(
    suspension_kernel PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/cpp/axle_dynamics"
    "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include"
)
target_link_options(suspension_kernel PRIVATE -static -static-libgcc -static-libstdc++)

# Flags are a one-to-one mapping of the previous direct-compile invocation,
# not a fresh choice: no semantic flag is added or dropped, and no
# floating-point semantic flag (for example `-ffp-contract`) is introduced.
target_compile_options(suspension_kernel PRIVATE ${KERNEL_COMPILE_OPTIONS})
target_link_options(suspension_kernel PRIVATE -fopenmp)
target_link_libraries(
    suspension_kernel PRIVATE
    "-Wl,--start-group"
    mb_contract
    mb_cases
    mb_solve_dynamic
    mb_solve_static
    mb_input
    mb_output
    mb_assembly
    mb_force
    mb_element
    mb_tire
    mb_joint
    mb_model
    mb_tire_pac2002
    mb_tire_fiala
    mb_tire_brush
    mb_tire_common
    mb_tire_state
    mb_linear
    mb_numeric
    mb_dual
    mb_config
    "-Wl,--end-group"
)

# OpenMP must be linked statically.  `find_package(OpenMP)` resolves MinGW's
# `libgomp.dll.a`, an import library, and linking that leaves the product
# depending on `libgomp-1.dll` at load time -- the previous direct-compile build
# embedded the runtime and produced a self-contained image.  So the static
# archive is located explicitly instead of relying on the default search order.
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    # Derive the runtime directory from the compiler itself: `<compiler>/../..`
    # is the toolchain prefix, whose `lib` holds `libgomp.a`.
    get_filename_component(_compiler_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
    get_filename_component(_toolchain_root "${_compiler_bin}/.." ABSOLUTE)
    set(_openmp_candidates "${_toolchain_root}/lib")
    find_library(
        SUSPENSION_KERNEL_OPENMP_STATIC
        NAMES libgomp.a
        PATHS ${_openmp_candidates}
        NO_DEFAULT_PATH
    )
    if(NOT SUSPENSION_KERNEL_OPENMP_STATIC)
        # Fall back to a direct glob: some toolchains name the archive without
        # the `lib` prefix in `find_library` terms.
        file(GLOB _libgomp "${_openmp_candidates}/libgomp.a")
        if(_libgomp)
            list(GET _libgomp 0 SUSPENSION_KERNEL_OPENMP_STATIC)
        endif()
    endif()
    if(NOT SUSPENSION_KERNEL_OPENMP_STATIC)
        message(
            FATAL_ERROR
            "a static OpenMP runtime (libgomp.a) is required so the shared "
            "library stays self-contained; linking the dynamic import library "
            "would add a runtime DLL dependency the previous build did not have. "
            "Looked in: ${_openmp_candidates}"
        )
    endif()
    message(STATUS "static OpenMP runtime: ${SUSPENSION_KERNEL_OPENMP_STATIC}")
    target_link_libraries(suspension_kernel PRIVATE ${SUSPENSION_KERNEL_OPENMP_STATIC})
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(NOT ipo_supported)
    message(FATAL_ERROR "link-time optimization is unavailable: ${ipo_error}")
endif()
set_property(
    TARGET suspension_kernel
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
)
set_property(
    TARGET mb_contract mb_cases mb_solve_dynamic mb_solve_static mb_output mb_assembly mb_force mb_element mb_tire mb_joint mb_model mb_tire_pac2002 mb_tire_fiala mb_tire_brush mb_tire_common mb_tire_state mb_linear mb_numeric mb_dual mb_config
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
)

set_property(
    TARGET suspension_kernel
    PROPERTY OUTPUT_NAME suspension_kernel
)
# MinGW prefixes shared libraries with `lib` by default; the loader resolves the
# undecorated platform name, so the prefix is suppressed explicitly.
set_property(TARGET suspension_kernel PROPERTY PREFIX "")
# The product stays in the build tree; the build driver copies it into
# `src/suspension_kernel/native`.  Linking straight into the package directory
# was tried first and the MinGW linker refused to create the file there.

# The contract layer is verified by a standalone executable rather than through
# the product C ABI: it must be exercised without bumping any ABI version.
add_executable(mb_contract_selftest cpp/tests/contract_selftest.cpp)
target_include_directories(
    mb_contract_selftest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/cpp/include"
)
target_compile_options(mb_contract_selftest PRIVATE ${KERNEL_COMPILE_OPTIONS})
target_link_options(mb_contract_selftest PRIVATE -fopenmp)
# Keep the self-test runnable without the MinGW runtime on PATH, so it can
# be invoked directly from CI and from the local gate scripts.
target_link_options(
    mb_contract_selftest PRIVATE -static -static-libgcc -static-libstdc++
)
target_link_libraries(mb_contract_selftest PRIVATE mb_contract)
