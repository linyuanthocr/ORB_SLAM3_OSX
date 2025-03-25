cmake_minimum_required(VERSION 3.16)
project(Sophus VERSION 1.1.0)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Enforce C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect macOS architecture
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

if (ARCH STREQUAL "arm64")
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=neoverse-n1")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -mcpu=neoverse-n1")
    message(STATUS "✅ Building for Apple Silicon (arm64)")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")
    message(STATUS "⚠️ Building for non-Apple Silicon: ${ARCH}")
endif()

# Compiler-specific settings
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -std=c++14 -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
    add_definitions("-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
endif()

# Add custom CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

# Find Eigen 3.4
find_package(Eigen3 3.4 REQUIRED)
if(EIGEN3_FOUND)
    set(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
    message(STATUS "✅ Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "❌ Eigen3 not found! Please install Eigen 3.4+ (e.g., via `brew install eigen` on macOS)")
endif()

# Create header-only library target
add_library(sophus INTERFACE)

set(SOPHUS_HEADER_FILES
  sophus/average.hpp
  sophus/common.hpp
  sophus/geometry.hpp
  sophus/interpolate.hpp
  sophus/interpolate_details.hpp
  sophus/num_diff.hpp
  sophus/rotation_matrix.hpp
  sophus/rxso2.hpp
  sophus/rxso3.hpp
  sophus/se2.hpp
  sophus/se3.hpp
  sophus/sim2.hpp
  sophus/sim3.hpp
  sophus/sim_details.hpp
  sophus/so2.hpp
  sophus/so3.hpp
  sophus/types.hpp
  sophus/velocities.hpp
  sophus/formatstring.hpp
)

# Ensure Eigen dependency
if(TARGET Eigen3::Eigen)
    target_link_libraries(sophus INTERFACE Eigen3::Eigen)
    set(Eigen3_DEPENDENCY "find_dependency (Eigen3 ${Eigen3_VERSION})")
else()
    target_include_directories(sophus SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})
endif()

# Associate target with include directories
target_include_directories(sophus INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

# Declare required C++14 features
target_compile_features(sophus INTERFACE
  cxx_auto_type
  cxx_decltype
  cxx_nullptr
  cxx_right_angle_brackets
  cxx_variadic_macros
  cxx_variadic_templates
)

# Enable tests (optional)
option(BUILD_TESTS "Build tests." ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Enable examples (optional)
option(BUILD_EXAMPLES "Build examples." ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Export package for use from the build tree
set(SOPHUS_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/sophus/cmake)

set_target_properties(sophus PROPERTIES EXPORT_NAME Sophus)

install(TARGETS sophus EXPORT SophusTargets)
install(EXPORT SophusTargets
  NAMESPACE Sophus::
  DESTINATION ${SOPHUS_CMAKE_EXPORT_DIR}
)

export(TARGETS sophus NAMESPACE Sophus:: FILE SophusTargets.cmake)
export(PACKAGE Sophus)

configure_package_config_file(
  SophusConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SophusConfig.cmake
  INSTALL_DESTINATION ${SOPHUS_CMAKE_EXPORT_DIR}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Ensure architecture independence for a header-only library
set(TEMP_SIZEOF_VOID_P ${CMAKE_SIZEOF_VOID_P})
unset(CMAKE_SIZEOF_VOID_P)

# Write version to file
write_basic_package_version_file(
  SophusConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Restore architecture dependence
set(CMAKE_SIZEOF_VOID_P ${TEMP_SIZEOF_VOID_P})

# Install cmake targets
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/SophusConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/SophusConfigVersion.cmake
  DESTINATION ${SOPHUS_CMAKE_EXPORT_DIR}
)

# Install header files
install(
  FILES ${SOPHUS_HEADER_FILES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sophus
)
