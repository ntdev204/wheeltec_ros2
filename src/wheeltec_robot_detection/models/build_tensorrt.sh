#!/bin/bash
# Build TensorRT engine from ONNX model on Jetson Orin Nano

ONNX_FILE="yolov8m.onnx"
ENGINE_FILE="yolov8m.engine"
WORKSPACE=4096

echo "Building TensorRT FP16 engine from $ONNX_FILE"
echo "This must be run on the target Jetson device!"

/usr/src/tensorrt/bin/trtexec \
    --onnx=$ONNX_FILE \
    --saveEngine=$ENGINE_FILE \
    --fp16 \
    --workspace=$WORKSPACE \
    --verbose

echo "TensorRT engine built: $ENGINE_FILE"
echo "Copy this file to: /home/jetson/wheeltec_ros2/src/wheeltec_robot_detection/models/"
