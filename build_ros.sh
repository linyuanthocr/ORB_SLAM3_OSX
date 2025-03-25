#!/bin/bash

# Script to build ORB-SLAM3 as a static library for iOS

# ** Configuration **
CONDA_ENV_NAME="orbslam_ios"  # Your Conda environment name
TOOLCHAIN_FILE="../ios.toolchain.cmake" # Path to your toolchain file (relative to script)
CMAKE_BUILD_TYPE="Release" # Release or Debug
BUILD_DIR="build_ios"  # Name of the build directory
ORB_SLAM3_ROOT=$(pwd) # Get the absolute path to the current directory (ORB-SLAM3 root)

# ** Check for errors and exit immediately **
set -e

# ** Activate Conda environment **
echo "Activating Conda environment: ${CONDA_ENV_NAME}"
source "$(conda info --base)/etc/profile.d/conda.sh" # Needed if conda init is not done
conda activate "${CONDA_ENV_NAME}" || { echo "ERROR: Conda environment '${CONDA_ENV_NAME}' not found.  Please create it first."; exit 1; }

# ** Create build directory **
echo "Creating build directory: ${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"  # -p creates the directory if it doesn't exist
cd "${BUILD_DIR}"

# ** Run CMake **
echo "Running CMake with toolchain file: ${TOOLCHAIN_FILE}"
cmake -DCMAKE_TOOLCHAIN_FILE="${ORB_SLAM3_ROOT}/${TOOLCHAIN_FILE}" -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" "${ORB_SLAM3_ROOT}"

# ** Build the library **
echo "Building the library (make)"
make -j4 #Use 4 cores to build

echo "Build complete! Library should be in: ${ORB_SLAM3_ROOT}/${BUILD_DIR}"

cd "${ORB_SLAM3_ROOT}" #Return to the project root when finished

exit 0
