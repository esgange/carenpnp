#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV_DIR="${ROOT}/third_party/.venv"

source_setup() {
  local had_errexit=0
  local had_nounset=0
  case $- in
    *e*) had_errexit=1; set +e ;;
  esac
  case $- in
    *u*) had_nounset=1; set +u ;;
  esac
  # shellcheck disable=SC1090
  source "$1"
  local status=$?
  if [ "${had_nounset}" -eq 1 ]; then
    set -u
  fi
  if [ "${had_errexit}" -eq 1 ]; then
    set -e
  fi
  return "${status}"
}

if [ -f /opt/ros/humble/setup.bash ]; then
  source_setup /opt/ros/humble/setup.bash
fi
if [ -f "${ROOT}/install/setup.bash" ]; then
  source_setup "${ROOT}/install/setup.bash"
fi
if [ -f "${VENV_DIR}/bin/activate" ]; then
  source_setup "${VENV_DIR}/bin/activate"
fi

echo "Checking ROS packages..."
for pkg in orbbec_camera orbbec_camera_msgs orbbec_description rviz2 dobot_rviz item_perception tray_perception; do
  ros2 pkg prefix "${pkg}" >/dev/null
  echo "  ok: ${pkg}"
done

echo "Checking Python imports..."
python - <<'PY'
import cv2
import aio_pika
import aiormq
from importlib.metadata import version
import numpy
import onnxruntime
import pytest
import torch
import torchvision
import ultralytics
import yaml
import sam2

print("  ok: python imports")
print("  torch:", torch.__version__)
print("  torchvision:", torchvision.__version__)
print("  onnxruntime:", onnxruntime.__version__)
print("  ultralytics:", ultralytics.__version__)
print("  aio-pika:", aio_pika.__version__)
print("  aiormq:", version("aiormq"))
print("  pytest:", pytest.__version__)
PY

command -v cell-external-bridge >/dev/null
echo "  ok: cell-external-bridge command"

echo "Checking bundled assets..."
test -f "${ROOT}/config/camera_bringup/orbbec_cameras.yaml"
test -f "${ROOT}/third_party/sam2/checkpoints/sam2.1_hiera_tiny.pt"
test -f "${ROOT}/third_party/yolo/checkpoints/yolo11n-seg.pt"
test -f "${ROOT}/third_party/manifest.yaml"
echo "  ok: config, checkpoints, manifest"

echo "Offline environment verification passed."
