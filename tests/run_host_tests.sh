#!/usr/bin/env sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$project_dir"
mkdir -p build

g++ -std=gnu++11 -Wall -Wextra -Werror \
  -Itests/host -IHexapodMega -x c++ \
  HexapodMega/HexapodMega.ino \
  HexapodMega/CommandProcessor.cpp \
  HexapodMega/ConfigStore.cpp \
  HexapodMega/GaitEngine.cpp \
  HexapodMega/HexapodController.cpp \
  HexapodMega/Kinematics.cpp \
  HexapodMega/PCA9685Driver.cpp \
  HexapodMega/ServoConfig.cpp \
  tests/host/stubs.cpp \
  tests/host/test_main.cpp \
  -o build/hexapod_host_test

./build/hexapod_host_test
echo "Host compile and smoke tests passed."
