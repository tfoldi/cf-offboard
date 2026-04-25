cmake_minimum_required(VERSION 3.20)
project(cf-offboard CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(_cfo_src "${CMAKE_CURRENT_SOURCE_DIR}/src")

# ----------------------------------------------------------------------------
# crazyflie-link-cpp dependency
#
# Auto-detected, in order:
#   1. Vendored copy at vendor/crazyflie-link-cpp/  (preferred — pinned source)
#   2. Installed package located via find_package(crazyflieLinkCpp)
#
# Override with -DCFOFFBOARD_LINK_SOURCE=vendor|system|auto (default: auto).
# The dependency is consumed only by the cfo_link target; vendor headers do
# not appear on any other target's include surface.
# ----------------------------------------------------------------------------

set(CFOFFBOARD_LINK_SOURCE "auto" CACHE STRING
    "Where to obtain crazyflie-link-cpp: auto | vendor | system")
set_property(CACHE CFOFFBOARD_LINK_SOURCE PROPERTY STRINGS auto vendor system)

set(_cfo_vendor_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/crazyflie-link-cpp")
set(_cfo_use_vendor OFF)

if(CFOFFBOARD_LINK_SOURCE STREQUAL "vendor")
    set(_cfo_use_vendor ON)
elseif(CFOFFBOARD_LINK_SOURCE STREQUAL "system")
    find_package(crazyflieLinkCpp REQUIRED)
elseif(CFOFFBOARD_LINK_SOURCE STREQUAL "auto")
    if(EXISTS "${_cfo_vendor_dir}/CMakeLists.txt")
        set(_cfo_use_vendor ON)
    else()
        find_package(crazyflieLinkCpp QUIET)
        if(NOT crazyflieLinkCpp_FOUND)
            message(FATAL_ERROR
                "crazyflie-link-cpp not found.\n"
                "  - Clone it into ${_cfo_vendor_dir}, or\n"
                "  - Install it and re-run CMake (auto-detected via find_package), or\n"
                "  - Pass -DCFOFFBOARD_LINK_SOURCE=system with CMAKE_PREFIX_PATH set.")
        endif()
    endif()
else()
    message(FATAL_ERROR
        "Invalid CFOFFBOARD_LINK_SOURCE='${CFOFFBOARD_LINK_SOURCE}' "
        "(expected: auto | vendor | system)")
endif()

if(_cfo_use_vendor)
    # Disable vendor's optional outputs we don't need (Python bindings require
    # a pybind11 submodule; the C++ examples are unrelated to our build).
    set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
    set(BUILD_CPP_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory("${_cfo_vendor_dir}" EXCLUDE_FROM_ALL)
    # Upstream uses directory-scoped include_directories(); add the include
    # path to the target's interface so consumers inherit it cleanly.
    target_include_directories(crazyflieLinkCpp INTERFACE
        $<BUILD_INTERFACE:${_cfo_vendor_dir}/include>)
    set(_cfo_link_dep crazyflieLinkCpp)
else()
    if(TARGET crazyflieLinkCpp::crazyflieLinkCpp)
        set(_cfo_link_dep crazyflieLinkCpp::crazyflieLinkCpp)
    else()
        set(_cfo_link_dep crazyflieLinkCpp)
    endif()
endif()

# ----------------------------------------------------------------------------
# MCAP — header-only logging library (foxglove/mcap).
# Compression dependencies (LZ4/ZSTD) are disabled; we write uncompressed.
# ----------------------------------------------------------------------------

set(_mcap_vendor_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/mcap")
if(NOT EXISTS "${_mcap_vendor_dir}/cpp/mcap/include/mcap/mcap.hpp")
    message(FATAL_ERROR
        "mcap not found at ${_mcap_vendor_dir}. Run scripts/bootstrap_vendor.sh.")
endif()

add_library(mcap_headers INTERFACE)
target_include_directories(mcap_headers INTERFACE
    "${_mcap_vendor_dir}/cpp/mcap/include")
target_compile_definitions(mcap_headers INTERFACE
    MCAP_COMPRESSION_NO_LZ4
    MCAP_COMPRESSION_NO_ZSTD)

# ----------------------------------------------------------------------------
# doctest — header-only test framework.
# ----------------------------------------------------------------------------

set(_doctest_vendor_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/doctest")
if(NOT EXISTS "${_doctest_vendor_dir}/doctest/doctest.h")
    message(FATAL_ERROR
        "doctest not found at ${_doctest_vendor_dir}. Run scripts/bootstrap_vendor.sh.")
endif()

add_library(doctest_headers INTERFACE)
target_include_directories(doctest_headers INTERFACE "${_doctest_vendor_dir}")

# ----------------------------------------------------------------------------
# State — value types + thread-safe StateStore. No vendor dependencies.
# ----------------------------------------------------------------------------

add_library(cfo_state STATIC
    src/state/state.cpp
)
target_include_directories(cfo_state PUBLIC ${_cfo_src})

# ----------------------------------------------------------------------------
# Crazyflie link adapter — only target that touches vendor headers.
# ----------------------------------------------------------------------------

add_library(cfo_link STATIC
    src/crazyflie/link/link.cpp
)
target_include_directories(cfo_link PUBLIC ${_cfo_src})
target_link_libraries(cfo_link PRIVATE ${_cfo_link_dep})

# ----------------------------------------------------------------------------
# Crazyflie LOG bring-up — synchronous TOC walk + block create/start.
# Narrow and explicit: just enough for slice 2's single hardcoded block.
# ----------------------------------------------------------------------------

add_library(cfo_log STATIC
    src/crazyflie/log/log_session.cpp
)
target_include_directories(cfo_log PUBLIC ${_cfo_src})
target_link_libraries(cfo_log PUBLIC cfo_link cfo_state)

# ----------------------------------------------------------------------------
# Safety — pure header-only supervisor (check_safety + types).
# ----------------------------------------------------------------------------

add_library(cfo_safety INTERFACE)
target_include_directories(cfo_safety INTERFACE ${_cfo_src})
target_link_libraries(cfo_safety INTERFACE cfo_state)

# ----------------------------------------------------------------------------
# Mission state machine — pure tick function, no IO. Drives the control
# loop's per-tick MissionCommand output.
# ----------------------------------------------------------------------------

add_library(cfo_mission STATIC
    src/mission/mission.cpp
)
target_include_directories(cfo_mission PUBLIC ${_cfo_src})
target_link_libraries(cfo_mission PUBLIC cfo_state cfo_safety)

# ----------------------------------------------------------------------------
# UI — hand-rolled ANSI terminal frontend. Observer of state, source of
# typed operator intents (no protocol awareness). Runs on its own thread.
# ----------------------------------------------------------------------------

add_library(cfo_ui STATIC
    src/ui/tui.cpp
)
target_include_directories(cfo_ui PUBLIC ${_cfo_src})
target_link_libraries(cfo_ui
    PUBLIC  cfo_state cfo_mission cfo_safety
    PRIVATE Threads::Threads)

# ----------------------------------------------------------------------------
# Logging — MCAP-backed async logger. mcap_headers is PRIVATE so MCAP types
# do not leak through cfo_logging's public surface. LogEvent references
# types from cfo_link / cfo_safety / cfo_state, hence those are PUBLIC.
# ----------------------------------------------------------------------------

add_library(cfo_logging STATIC
    src/logging/mcap_logger.cpp
    src/logging/console.cpp
)
target_include_directories(cfo_logging PUBLIC ${_cfo_src})
target_link_libraries(cfo_logging
    PUBLIC  cfo_link cfo_safety cfo_state cfo_mission
    PRIVATE mcap_headers Threads::Threads)
# cfo_logging includes <ui/types.hpp> for EventLog, but those are header-only
# value types — no link-time symbols needed. Include path is already covered
# by the `_cfo_src` PUBLIC directory above.

# ----------------------------------------------------------------------------
# Control loop — fixed-rate setpoint streamer for slice (a)'s hover demo.
# Depends on cfo_logging because control_loop.hpp signatures reference
# MCAPLogger and the loop body emits SetpointCommandEvent / SafetyEvent.
# ----------------------------------------------------------------------------

add_library(cfo_control STATIC
    src/control/control_loop.cpp
)
target_include_directories(cfo_control PUBLIC ${_cfo_src})
target_link_libraries(cfo_control
    PUBLIC  cfo_link cfo_state cfo_safety cfo_mission cfo_logging
    PRIVATE Threads::Threads)

# ----------------------------------------------------------------------------
# Application — startup, RX thread, graceful shutdown.
# ----------------------------------------------------------------------------

add_executable(cf-offboard src/app/main.cpp)
target_link_libraries(cf-offboard PRIVATE
    cfo_link cfo_log cfo_logging cfo_state cfo_control cfo_safety cfo_mission
    cfo_ui Threads::Threads)

# ----------------------------------------------------------------------------
# Offline tools — debugging and analysis utilities. Self-contained; do not
# link cfo_logging (each defines MCAP_IMPLEMENTATION privately).
# ----------------------------------------------------------------------------

add_executable(cfo_inspect tools/cfo_inspect.cpp)
target_link_libraries(cfo_inspect PRIVATE mcap_headers)

# ----------------------------------------------------------------------------
# Smallest possible compile check for the adapter.
# Building this target verifies that cfo_link compiles and links against the
# vendor library. It does not require a Crazyradio to be attached.
# ----------------------------------------------------------------------------

add_executable(cfo_link_compile_check tests/link_compile_check.cpp)
target_link_libraries(cfo_link_compile_check PRIVATE cfo_link)

# ----------------------------------------------------------------------------
# Unit tests — doctest-based. One executable, one TEST_CASE file per topic.
# ----------------------------------------------------------------------------

enable_testing()
add_executable(cfo_tests
    tests/doctest_main.cpp
    tests/test_protocol.cpp
    tests/test_log_protocol.cpp
    tests/test_state.cpp
    tests/test_setpoint.cpp
    tests/test_safety.cpp
    tests/test_mission.cpp
)
target_include_directories(cfo_tests PRIVATE ${_cfo_src})
target_link_libraries(cfo_tests PRIVATE
    cfo_link cfo_log cfo_state cfo_safety cfo_mission cfo_control doctest_headers)
add_test(NAME cfo_tests COMMAND cfo_tests)
