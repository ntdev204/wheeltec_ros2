"""TensorRT detector for YOLOv8 models on Jetson — performance-optimized.

Optimization layers applied:
  1. Preprocess: cv2.dnn.blobFromImage (single C++ pass: resize+BGR→RGB+norm)
     written directly into pinned TRT input buffer (zero extra copy).
  2. Inference: async CUDA stream — H2D / execute_async_v3 / D2H pipelined.
  3. Postprocess: fully vectorized NumPy (no Python for-loops over boxes).
  4. NMS: cv2.dnn.NMSBoxes — C++ batched NMS, replaces O(n²) Python loop.
  5. Memory: all buffers pre-allocated once; no per-frame malloc.
"""

import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
from typing import List, Tuple, Dict
import cv2

from . import BaseDetector

cuda.init()


class TensorRTDetector(BaseDetector):
    """TensorRT YOLOv8 detector — zero-copy preprocess, async inference, vectorized NMS."""

    def __init__(self, model_path: str, input_size: Tuple[int, int],
                 confidence_threshold: float = 0.5, nms_threshold: float = 0.45,
                 class_names: List[str] = None):
        super().__init__(model_path, input_size, confidence_threshold, nms_threshold)
        self.class_names = class_names or []
        self.engine = None
        self.context = None
        self.inputs = []
        self.outputs = []
        self.stream = None

        # CUDA context — bound to constructor thread.
        # SingleThreadedExecutor guarantees all callbacks run on this same thread.
        self.cuda_device = cuda.Device(0)
        self.cuda_ctx = self.cuda_device.make_context()

        # Pre-allocate reusable letterbox canvas (pinned memory → zero-copy into TRT buffer)
        target_w, target_h = self.input_size
        # Pinned HWC float32 staging buffer for letterbox output
        self._letterbox_buf: np.ndarray = cuda.pagelocked_empty(
            (target_h, target_w, 3), dtype=np.float32
        )
        self._letterbox_buf[:] = 114.0 / 255.0  # default grey fill

        self.load_model()

    # ── Model loading ─────────────────────────────────────────────────────────

    def load_model(self) -> None:
        """Deserialize TensorRT engine and allocate pinned I/O buffers."""
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

        with open(self.model_path, 'rb') as f:
            engine_data = f.read()

        runtime = trt.Runtime(TRT_LOGGER)
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            size = trt.volume(self.engine.get_tensor_shape(name))
            if size < 0:
                size = abs(size)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))

            host_mem   = cuda.pagelocked_empty(size, dtype)  # pinned
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            self.context.set_tensor_address(name, int(device_mem))

            entry = {'host': host_mem, 'device': device_mem, 'name': name}
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append(entry)
            else:
                self.outputs.append(entry)

    # ── Preprocessing (CPU → pinned, zero extra alloc) ─────────────────────

    def preprocess(self, image: np.ndarray) -> None:
        """
        Letterbox + normalize + CHW — written directly into pinned TRT input buffer.

        Strategy:
          - cv2.dnn.blobFromImage: single C++ op (resize + BGR→RGB + /255.0 + CHW)
            replaces 5 separate Python/NumPy allocations.
          - Output is written in-place into the TRT pinned input buffer (self.inputs[0]['host'])
            so memcpy_htod_async has zero staging overhead.

        Returns None — result already in self.inputs[0]['host'].
        """
        h, w = image.shape[:2]
        target_w, target_h = self.input_size

        # Letterbox scale + padding
        scale  = min(target_w / w, target_h / h)
        new_w  = int(round(w * scale))
        new_h  = int(round(h * scale))
        pad_w  = (target_w - new_w) // 2
        pad_h  = (target_h - new_h) // 2

        # Resize in one C++ call
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Single-pass: BGR→RGB + normalize → [1, 3, new_h, new_w] float32
        blob = cv2.dnn.blobFromImage(
            resized,
            scalefactor=1.0 / 255.0,
            size=(new_w, new_h),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
            ddepth=cv2.CV_32F,
        )  # shape [1, 3, new_h, new_w]

        # Write into pinned TRT buffer (1CHW view of the pinned array)
        inp = self.inputs[0]['host'].reshape(1, 3, target_h, target_w)
        inp[:] = 114.0 / 255.0  # reset grey fill
        inp[:, :, pad_h:pad_h + new_h, pad_w:pad_w + new_w] = blob[:, :, :new_h, :new_w]

    # ── Inference (async CUDA stream) ──────────────────────────────────────

    def inference(self, _=None) -> np.ndarray:
        """
        Async TRT inference on CUDA stream.

        Preprocess has already populated self.inputs[0]['host'].
        H2D → execute_async_v3 → D2H are all enqueued on the same stream,
        then stream.synchronize() blocks until GPU is done.
        """
        cuda.memcpy_htod_async(self.inputs[0]['device'],  self.inputs[0]['host'],  self.stream)
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.outputs[0]['host'], self.outputs[0]['device'], self.stream)
        self.stream.synchronize()
        return self.outputs[0]['host']

    # ── Postprocess (fully vectorized, no Python for-loops) ────────────────

    def postprocess(self, output: np.ndarray, original_shape: Tuple[int, int]) -> List[Dict]:
        """
        Vectorized YOLOv8 postprocess + NMS.

        Replaces O(n) Python for-loop with pure NumPy ops.
        NMS uses cv2.dnn.NMSBoxes (C++ impl of batched IoU suppression).
        """
        target_w, target_h = self.input_size
        orig_h, orig_w = original_shape

        scale   = min(target_w / orig_w, target_h / orig_h)
        pad_w   = (target_w - int(round(orig_w * scale))) // 2
        pad_h   = (target_h - int(round(orig_h * scale))) // 2

        # Reshape [84 * 8400] → [8400, 84]
        preds = output.reshape(84, -1).T           # [8400, 84]

        # ── Vectorized confidence filter ─────────────────────────────────
        scores     = preds[:, 4:]                  # [8400, num_cls]
        class_ids  = np.argmax(scores, axis=1)     # [8400]
        confs      = scores[np.arange(len(scores)), class_ids]  # [8400]

        mask = confs >= self.confidence_threshold
        if not np.any(mask):
            return []

        preds      = preds[mask]
        class_ids  = class_ids[mask]
        confs      = confs[mask]

        # ── Vectorized bbox (cx,cy,w,h) → (x1,y1,x2,y2) + unpad + unscale ─
        cx, cy, bw, bh = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]

        x1 = ((cx - bw / 2 - pad_w) / scale).astype(np.float32)
        y1 = ((cy - bh / 2 - pad_h) / scale).astype(np.float32)
        x2 = ((cx + bw / 2 - pad_w) / scale).astype(np.float32)
        y2 = ((cy + bh / 2 - pad_h) / scale).astype(np.float32)

        # Clip to image bounds
        x1 = np.clip(x1, 0, orig_w).astype(int)
        y1 = np.clip(y1, 0, orig_h).astype(int)
        x2 = np.clip(x2, 0, orig_w).astype(int)
        y2 = np.clip(y2, 0, orig_h).astype(int)

        # ── C++ batched NMS via cv2.dnn.NMSBoxes ─────────────────────────
        # expects [x, y, w, h] format
        boxes_xywh = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(
            boxes_xywh,
            confs.tolist(),
            self.confidence_threshold,
            self.nms_threshold,
        )

        if len(indices) == 0:
            return []

        idx = np.array(indices).flatten()

        # ── Build result list (only kept boxes, vectorized slice) ─────────
        return [
            {
                'class_id':   int(class_ids[i]),
                'class_name': (self.class_names[class_ids[i]]
                               if class_ids[i] < len(self.class_names)
                               else f'class_{class_ids[i]}'),
                'confidence': float(confs[i]),
                'bbox':       [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
            }
            for i in idx
        ]

    # ── Fused pipeline ─────────────────────────────────────────────────────

    def detect(self, image: np.ndarray) -> List[Dict]:
        """
        Fused preprocess → inference → postprocess.

        Overrides BaseDetector.detect() to skip intermediate return value
        of preprocess() — output already lives in pinned TRT input buffer.
        """
        original_shape = image.shape[:2]
        self.preprocess(image)          # writes directly into pinned buffer
        raw = self.inference()          # H2D + TRT async + D2H
        return self.postprocess(raw, original_shape)
