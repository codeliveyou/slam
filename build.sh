#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

require_file() {
  local file="$1"
  local hint="$2"
  if [[ ! -f "$file" ]]; then
    echo "ERROR: Missing required file: $file"
    echo "Hint: $hint"
    exit 1
  fi
}

check_prerequisites() {
  require_file "$ROOT_DIR/Thirdparty/DBoW2/DUtils/Timestamp.cpp" \
    "Thirdparty sources look incomplete. Re-clone the repo and ensure all Thirdparty files are present."
  require_file "$ROOT_DIR/Thirdparty/g2o/g2o/stuff/timeutil.cpp" \
    "Thirdparty sources look incomplete. Re-clone the repo and ensure all Thirdparty files are present."

  if ! cmake --find-package -DNAME=Pangolin -DLANGUAGE=CXX -DMODE=EXIST >/dev/null 2>&1; then
    echo "ERROR: Pangolin was not found by CMake."
    echo "Hint: install Pangolin development files and/or set Pangolin_DIR to PangolinConfig.cmake."
    exit 1
  fi
}

check_prerequisites

echo "Configuring and building Thirdparty/DBoW2 ..."
cd "$ROOT_DIR/Thirdparty/DBoW2"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building Thirdparty/g2o ..."
cd "$ROOT_DIR/Thirdparty/g2o"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configuring and building Thirdparty/Sophus ..."
cd "$ROOT_DIR/Thirdparty/Sophus"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Uncompress vocabulary ..."
cd "$ROOT_DIR/Vocabulary"
tar -xf ORBvoc.txt.tar.gz

echo "Configuring and building ORB_SLAM3 ..."
cd "$ROOT_DIR"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
