#!/bin/bash
# Build TensorRT FP16 engine from ONNX — must run on target Jetson device.
set -euo pipefail

TRTEXEC="/usr/src/tensorrt/bin/trtexec"
# Always resolve paths relative to THIS script's directory (not CWD)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODEL="${1:-yolov8s}"   # default: yolov8s  (pass "yolov8m" to override)
ONNX_FILE="${SCRIPT_DIR}/${MODEL}.onnx"
ENGINE_FILE="${SCRIPT_DIR}/${MODEL}.engine"
WORKSPACE=4096

echo "=========================================="
echo " TensorRT FP16 Engine Builder"
echo " Model  : ${MODEL}"
echo " ONNX   : ${ONNX_FILE}"
echo " Engine : ${ENGINE_FILE}"
echo " Device : Jetson Orin Nano (SM 8.7)"
echo "=========================================="

# Sanity checks
if [[ ! -f "${ONNX_FILE}" ]]; then
    echo "[ERROR] ONNX file not found: ${ONNX_FILE}"
    echo "  Export it first: python3 -c \"from ultralytics import YOLO; YOLO('${MODEL}.pt').export(format='onnx', imgsz=640)\""
    exit 1
fi

if [[ ! -x "${TRTEXEC}" ]]; then
    echo "[ERROR] trtexec not found at ${TRTEXEC}"
    exit 1
fi

echo ""
echo "[INFO] Starting engine build (this takes 5–15 minutes)..."
echo ""

"${TRTEXEC}" \
    --onnx="${ONNX_FILE}" \
    --saveEngine="${ENGINE_FILE}" \
    --fp16 \
    --memPoolSize=workspace:${WORKSPACE}

echo ""
echo "[OK] Engine saved: ${ENGINE_FILE}"
echo "[INFO] Engine is optimized for THIS device — do not copy to other hardware."
