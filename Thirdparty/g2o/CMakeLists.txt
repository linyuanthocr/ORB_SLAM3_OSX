cmake_minimum_required(VERSION 3.16)
project(g2o CXX C)

set(CMAKE_MACOSX_RPATH 1)

# Detect architecture
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

if (ARCH STREQUAL "arm64")
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -mcpu=neoverse-n1")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -mcpu=neoverse-n1")
    message(STATUS "✅ Building for Apple Silicon (arm64)")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
    message(STATUS "⚠️  Building for non-Apple Silicon: ${ARCH}")
endif()

# Default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "BUILD TYPE: ${CMAKE_BUILD_TYPE}")

set(G2O_LIB_TYPE STATIC)

# Define output directories
if(WIN32)
    set(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
else()
    set(g2o_LIBRARY_OUTPUT_DIRECTORY ${g2o_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
endif()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for CMake scripts
list(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS
if(UNIX)
    add_definitions(-DUNIX)
    message(STATUS "Compiling on Unix")
endif()

# OpenMP (Experimental)
find_package(OpenMP)
set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
if(OPENMP_FOUND AND G2O_USE_OPENMP)
    set(G2O_OPENMP 1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    message(STATUS "Compiling with OpenMP support")
endif()

# Activate warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -W")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W")

# Find Eigen3
find_package(Eigen3 3.1.0 REQUIRED)
if(EIGEN3_FOUND)
    set(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
else()
    set(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
endif()

# Generate config.h
set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)

# Set up include directories
include_directories(
    ${g2o_SOURCE_DIR}/core
    ${g2o_SOURCE_DIR}/types
    ${g2o_SOURCE_DIR}/stuff
    ${G2O_EIGEN3_INCLUDE}
)

# Include the subdirectories
add_library(g2o ${G2O_LIB_TYPE}
    # Types
    g2o/types/types_sba.h
    g2o/types/types_six_dof_expmap.h
    g2o/types/types_sba.cpp
    g2o/types/types_six_dof_expmap.cpp
    g2o/types/types_seven_dof_expmap.cpp
    g2o/types/types_seven_dof_expmap.h
    g2o/types/se3quat.h
    g2o/types/se3_ops.h
    g2o/types/se3_ops.hpp

    # Core
    g2o/core/base_edge.h
    g2o/core/base_binary_edge.h
    g2o/core/hyper_graph_action.cpp
    g2o/core/base_binary_edge.hpp
    g2o/core/hyper_graph_action.h
    g2o/core/base_multi_edge.h
    g2o/core/hyper_graph.cpp
    g2o/core/base_multi_edge.hpp
    g2o/core/hyper_graph.h
    g2o/core/base_unary_edge.h
    g2o/core/linear_solver.h
    g2o/core/base_unary_edge.hpp
    g2o/core/marginal_covariance_cholesky.cpp
    g2o/core/base_vertex.h
    g2o/core/marginal_covariance_cholesky.h
    g2o/core/base_vertex.hpp
    g2o/core/matrix_structure.cpp
    g2o/core/batch_stats.cpp
    g2o/core/matrix_structure.h
    g2o/core/batch_stats.h
    g2o/core/openmp_mutex.h
    g2o/core/block_solver.h
    g2o/core/block_solver.hpp
    g2o/core/parameter.cpp
    g2o/core/parameter.h
    g2o/core/cache.cpp
    g2o/core/cache.h
    g2o/core/optimizable_graph.cpp
    g2o/core/optimizable_graph.h
    g2o/core/solver.cpp
    g2o/core/solver.h
    g2o/core/creators.h
    g2o/core/optimization_algorithm_factory.cpp
    g2o/core/estimate_propagator.cpp
    g2o/core/optimization_algorithm_factory.h
    g2o/core/estimate_propagator.h
    g2o/core/factory.cpp
    g2o/core/optimization_algorithm_property.h
    g2o/core/factory.h
    g2o/core/sparse_block_matrix.h
    g2o/core/sparse_optimizer.cpp
    g2o/core/sparse_block_matrix.hpp
    g2o/core/sparse_optimizer.h
    g2o/core/hyper_dijkstra.cpp
    g2o/core/hyper_dijkstra.h
    g2o/core/parameter_container.cpp
    g2o/core/parameter_container.h
    g2o/core/optimization_algorithm.cpp
    g2o/core/optimization_algorithm.h
    g2o/core/optimization_algorithm_with_hessian.cpp
    g2o/core/optimization_algorithm_with_hessian.h
    g2o/core/optimization_algorithm_levenberg.cpp
    g2o/core/optimization_algorithm_levenberg.h
    g2o/core/optimization_algorithm_gauss_newton.cpp
    g2o/core/optimization_algorithm_gauss_newton.h
    g2o/core/jacobian_workspace.cpp
    g2o/core/jacobian_workspace.h
    g2o/core/robust_kernel.cpp
    g2o/core/robust_kernel.h
    g2o/core/robust_kernel_factory.cpp
    g2o/core/robust_kernel_factory.h
    g2o/core/robust_kernel_impl.cpp
    g2o/core/robust_kernel_impl.h

    # Stuff
    g2o/stuff/string_tools.h
    g2o/stuff/color_macros.h
    g2o/stuff/macros.h
    g2o/stuff/timeutil.cpp
    g2o/stuff/misc.h
    g2o/stuff/timeutil.h
    g2o/stuff/os_specific.c
    g2o/stuff/os_specific.h
    g2o/stuff/string_tools.cpp
    g2o/stuff/property.cpp
    g2o/stuff/property.h
)
