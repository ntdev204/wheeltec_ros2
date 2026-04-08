#!/bin/bash
# Build TensorRT engine from ONNX model on Jetson Orin Nano

ONNX_FILE="yolov8m.onnx"
ENGINE_FILE="yolov8m_int8.engine"
WORKSPACE=4096

echo "Building TensorRT INT8 engine from $ONNX_FILE"
echo "This must be run on the target Jetson device!"

trtexec \
    --onnx=$ONNX_FILE \
    --saveEngine=$ENGINE_FILE \
    --int8 \
    --workspace=$WORKSPACE \
    --fp16 \
    --verbose \
    --inputIOFormats=fp16:chw \
    --outputIOFormats=fp16:chw

echo "TensorRT engine built: $ENGINE_FILE"
echo "Copy this file to: /home/jetson/wheeltec_ros2/src/wheeltec_robot_detection/models/"
