"""TensorRT detector for YOLOv8 models on Jetson."""

import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from typing import List, Tuple, Dict
import cv2

from . import BaseDetector


class TensorRTDetector(BaseDetector):
    """TensorRT-based YOLOv8 detector optimized for Jetson."""

    def __init__(self, model_path: str, input_size: Tuple[int, int],
                 confidence_threshold: float = 0.5, nms_threshold: float = 0.45,
                 class_names: List[str] = None):
        """
        Initialize TensorRT detector.

        Args:
            model_path: Path to TensorRT engine file (.engine)
            input_size: Input image size (width, height)
            confidence_threshold: Confidence threshold for detections
            nms_threshold: NMS IOU threshold
            class_names: List of class names
        """
        super().__init__(model_path, input_size, confidence_threshold, nms_threshold)
        self.class_names = class_names or []
        self.engine = None
        self.context = None
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = None

        self.load_model()

    def load_model(self) -> None:
        """Load TensorRT engine."""
        # Create logger
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

        # Load engine
        with open(self.model_path, 'rb') as f:
            engine_data = f.read()

        runtime = trt.Runtime(TRT_LOGGER)
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.stream = cuda.Stream()

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            size = trt.volume(self.engine.get_tensor_shape(name))
            if size < 0:
                size = abs(size) # Default fallback for dynamic variables if any
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))

            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            # Assign memory to execution context for this tensor
            self.context.set_tensor_address(name, int(device_mem))

            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append({'host': host_mem, 'device': device_mem, 'name': name})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem, 'name': name})

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for YOLOv8 TensorRT inference.

        Args:
            image: Input image (BGR format)

        Returns:
            Preprocessed image tensor
        """
        # Letterbox resize
        img = self._letterbox(image, self.input_size)

        # Convert BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Normalize to [0, 1]
        img = img.astype(np.float32) / 255.0

        # HWC to CHW
        img = np.transpose(img, (2, 0, 1))

        # Add batch dimension
        img = np.expand_dims(img, axis=0)

        # Ensure contiguous
        img = np.ascontiguousarray(img)

        return img

    def _letterbox(self, image: np.ndarray, new_shape: Tuple[int, int]) -> np.ndarray:
        """
        Letterbox resize maintaining aspect ratio.

        Args:
            image: Input image
            new_shape: Target size (width, height)

        Returns:
            Resized image with padding
        """
        shape = image.shape[:2]  # current shape [height, width]

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[1], new_shape[1] / shape[0])

        # Compute padding
        new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
        dw = new_shape[0] - new_unpad[0]
        dh = new_shape[1] - new_unpad[1]

        dw /= 2
        dh /= 2

        if shape[::-1] != new_unpad:
            image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)

        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

        image = cv2.copyMakeBorder(image, top, bottom, left, right,
                                   cv2.BORDER_CONSTANT, value=(114, 114, 114))

        return image

    def inference(self, preprocessed_image: np.ndarray) -> np.ndarray:
        """
        Run TensorRT inference.

        Args:
            preprocessed_image: Preprocessed image tensor

        Returns:
            Raw model output
        """
        # Copy input to device
        np.copyto(self.inputs[0]['host'], preprocessed_image.ravel())
        cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)

        # Run inference
        self.context.execute_async_v3(stream_handle=self.stream.handle)

        # Copy output to host
        cuda.memcpy_dtoh_async(self.outputs[0]['host'], self.outputs[0]['device'], self.stream)

        # Synchronize
        self.stream.synchronize()

        return self.outputs[0]['host']

    def postprocess(self, output: np.ndarray, original_shape: Tuple[int, int]) -> List[Dict]:
        """
        Postprocess YOLOv8 output.

        Args:
            output: Raw model output
            original_shape: Original image shape (height, width)

        Returns:
            List of detections
        """
        # Reshape output (YOLOv8 format: [batch, 84, 8400] for COCO)
        # 84 = 4 (bbox) + 80 (classes)
        output = output.reshape(1, 84, -1)
        output = np.transpose(output, (0, 2, 1))  # [batch, 8400, 84]

        detections = []

        for pred in output[0]:
            # Extract bbox and scores
            bbox = pred[:4]
            scores = pred[4:]

            # Get class with max score
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence < self.confidence_threshold:
                continue

            # Convert from center format to corner format
            x_center, y_center, width, height = bbox
            x_min = int(x_center - width / 2)
            y_min = int(y_center - height / 2)
            x_max = int(x_center + width / 2)
            y_max = int(y_center + height / 2)

            # Scale to original image size
            scale_x = original_shape[1] / self.input_size[0]
            scale_y = original_shape[0] / self.input_size[1]

            x_min = int(x_min * scale_x)
            y_min = int(y_min * scale_y)
            x_max = int(x_max * scale_x)
            y_max = int(y_max * scale_y)

            detections.append({
                'class_id': int(class_id),
                'class_name': self.class_names[class_id] if class_id < len(self.class_names) else f'class_{class_id}',
                'confidence': float(confidence),
                'bbox': [x_min, y_min, x_max, y_max]
            })

        # Apply NMS
        detections = self._nms(detections)

        return detections

    def _nms(self, detections: List[Dict]) -> List[Dict]:
        """
        Apply Non-Maximum Suppression.

        Args:
            detections: List of detections

        Returns:
            Filtered detections
        """
        if len(detections) == 0:
            return []

        boxes = np.array([d['bbox'] for d in detections])
        scores = np.array([d['confidence'] for d in detections])

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h

            iou = inter / (areas[i] + areas[order[1:]] - inter)

            inds = np.where(iou <= self.nms_threshold)[0]
            order = order[inds + 1]

        return [detections[i] for i in keep]
