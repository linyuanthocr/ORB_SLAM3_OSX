cmake_minimum_required(VERSION 3.16)

project(ORB_SLAM3_iOS)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Force Apple Silicon (arm64) for iOS
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

if (ARCH STREQUAL "arm64")
    set(CMAKE_OSX_ARCHITECTURES arm64)
    message(STATUS "✅ Building for Apple Silicon (arm64)")
else()
    message(STATUS "⚠️ Building for a different architecture: ${ARCH}")
endif()

# Compiler flags (removing -march=native for cross-compilation)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Find dependencies
find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(Boost REQUIRED serialization)
# Tell CMake where to find your PangolinConfig.cmake
set(CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/build")
find_package(Pangolin REQUIRED)
find_package(OpenSSL REQUIRED)


# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
)

# Set library output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Force g2o to build as a static library
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)

# Add g2o as a subdirectory
add_subdirectory(Thirdparty/g2o)

# Source files (excluding GUI-dependent files for iOS)
set(SLAM_SOURCES
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Atlas.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/ImuTypes.cc
    src/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/OptimizableTypes.cpp
    src/MLPnPsolver.cpp
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/Config.cc
    src/Settings.cc
    include/System.h
    include/Tracking.h
    include/LocalMapping.h
    include/LoopClosing.h
    include/ORBextractor.h
    include/ORBmatcher.h
    include/FrameDrawer.h
    include/Converter.h
    include/MapPoint.h
    include/KeyFrame.h
    include/Atlas.h
    include/Map.h
    include/MapDrawer.h
    include/Optimizer.h
    include/Frame.h
    include/KeyFrameDatabase.h
    include/Sim3Solver.h
    include/ImuTypes.h
    include/G2oTypes.h
    include/CameraModels/GeometricCamera.h
    include/CameraModels/Pinhole.h
    include/CameraModels/KannalaBrandt8.h
    include/OptimizableTypes.h
    include/MLPnPsolver.h
    include/GeometricTools.h
    include/TwoViewReconstruction.h
    include/SerializationUtils.h
    include/Config.h
    include/Settings.h
)

# Remove GUI-dependent Viewer for iOS
if(NOT IOS)
    list(APPEND SLAM_SOURCES src/Viewer.cc include/Viewer.h)
endif()

# Create a static library
add_library(${PROJECT_NAME} STATIC ${SLAM_SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${Pangolin_LIBRARIES}
    ${Boost_LIBRARIES}
    g2o
    OpenSSL::Crypto
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.a # Ensure static linking
)

# Linker settings for iOS
if(IOS)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        XCODE_ATTRIBUTE_ONLY_ACTIVE_ARCH "YES"
        XCODE_ATTRIBUTE_ENABLE_BITCODE "NO"
        XCODE_ATTRIBUTE_VALID_ARCHS "arm64"
    )
endif()

message(STATUS "Pangolin libs: ${Pangolin_LIBRARIES}")
message(STATUS "Pangolin header files: ${Pangolin_INCLUDE_DIRS}")



# ============================
# Monocular Examples
# ============================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_euroc_osx Examples/Monocular/mono_euroc_osx.cc)
target_link_libraries(mono_euroc_osx ${PROJECT_NAME})

add_executable(mono_tum_vi Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})


# ============================
# Monocular-Inertial Examples
# ============================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})