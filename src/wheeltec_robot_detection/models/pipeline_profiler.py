#!/usr/bin/env python3
"""
Full pipeline profiler — runs OUTSIDE ROS to measure each stage:
  preprocess / TRT inference / postprocess / preview_encode / cv_bridge_decode
Usage: python3 pipeline_profiler.py [engine_path]
"""
import sys, time, os
import numpy as np
import cv2
import pycuda.driver as cuda
import tensorrt as trt

ENGINE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__), "yolov8s.engine"
)
N = 100
INPUT_SIZE = (640, 640)

# ── Setup ────────────────────────────────────────────────────────────────────
cuda.init()
dev = cuda.Device(0)
ctx = dev.make_context()
print(f"GPU: {dev.name()}")
print(f"Engine: {ENGINE}\n")

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
with open(ENGINE, "rb") as f:
    data = f.read()
rt = trt.Runtime(TRT_LOGGER)
engine = rt.deserialize_cuda_engine(data)
context = engine.create_execution_context()
stream = cuda.Stream()

inputs, outputs = [], []
for i in range(engine.num_io_tensors):
    name = engine.get_tensor_name(i)
    size = abs(trt.volume(engine.get_tensor_shape(name)))
    dtype = trt.nptype(engine.get_tensor_dtype(name))
    h = cuda.pagelocked_empty(size, dtype)
    d = cuda.mem_alloc(h.nbytes)
    context.set_tensor_address(name, int(d))
    if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
        inputs.append({"h": h, "d": d})
    else:
        outputs.append({"h": h, "d": d})

# Pinned letterbox buffer (filled once)
inp_buf = inputs[0]["h"].reshape(1, 3, INPUT_SIZE[1], INPUT_SIZE[0])
inp_buf[:] = 114.0 / 255.0

# Warmup
for _ in range(10):
    cuda.memcpy_htod_async(inputs[0]["d"], inputs[0]["h"], stream)
    context.execute_async_v3(stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(outputs[0]["h"], outputs[0]["d"], stream)
    stream.synchronize()

def bench(fn, label):
    t0 = time.perf_counter()
    for _ in range(N):
        fn()
    ms = (time.perf_counter() - t0) / N * 1000
    fps = 1000 / ms
    print(f"  {label:<42} {ms:7.2f} ms  ({fps:6.1f} FPS)")
    return ms

# ── Fake frames ──────────────────────────────────────────────────────────────
frame_480p  = np.random.randint(0, 255, (480,  640, 3), dtype=np.uint8)
frame_720p  = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)

print("=" * 65)
print(" STAGE BREAKDOWN (640x480 input → typical Astra S)")
print("=" * 65)

# 1. cv_bridge decode equivalent
def cv_bridge_decode():
    raw = frame_480p.tobytes()
    return np.frombuffer(raw, dtype=np.uint8).reshape(frame_480p.shape)

t_bridge = bench(cv_bridge_decode, "[1] cv_bridge imgmsg→cv2 (480p)")

# 2. Preprocess WITHOUT inp reset (fixed version)
def preprocess_fixed():
    h, w = frame_480p.shape[:2]
    tw, th = INPUT_SIZE
    scale = min(tw / w, th / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    pw, ph = (tw - nw) // 2, (th - nh) // 2
    resized = cv2.resize(frame_480p, (nw, nh), interpolation=cv2.INTER_LINEAR)
    blob = cv2.dnn.blobFromImage(resized, 1/255., (nw, nh), (0,0,0), True, False, cv2.CV_32F)
    inp_buf[:, :, ph:ph+nh, pw:pw+nw] = blob[:, :, :nh, :nw]  # NO reset

t_pre = bench(preprocess_fixed, "[2] Preprocess fixed (no inp reset)")

# 3. Preprocess WITH inp reset (old buggy version)
def preprocess_old():
    h, w = frame_480p.shape[:2]
    tw, th = INPUT_SIZE
    scale = min(tw / w, th / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    pw, ph = (tw - nw) // 2, (th - nh) // 2
    resized = cv2.resize(frame_480p, (nw, nh), interpolation=cv2.INTER_LINEAR)
    blob = cv2.dnn.blobFromImage(resized, 1/255., (nw, nh), (0,0,0), True, False, cv2.CV_32F)
    inp_buf[:] = 114.0 / 255.0   # BUG: 3MB write every frame
    inp_buf[:, :, ph:ph+nh, pw:pw+nw] = blob[:, :, :nh, :nw]

t_pre_old = bench(preprocess_old, "[2b] Preprocess OLD (with inp reset)")

# 4. TRT inference
def trt_inference():
    cuda.memcpy_htod_async(inputs[0]["d"], inputs[0]["h"], stream)
    context.execute_async_v3(stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(outputs[0]["h"], outputs[0]["d"], stream)
    stream.synchronize()

t_trt = bench(trt_inference, "[3] TRT inference (H2D+exec+D2H)")

# 5. Postprocess with low detections (realistic: few objects)
few_boxes_output = np.random.rand(84 * 8400).astype(np.float32) * 0.1  # low conf → nearly empty

def postprocess_few():
    preds = few_boxes_output.reshape(84, -1).T
    scores = preds[:, 4:]
    class_ids = np.argmax(scores, axis=1)
    confs = scores[np.arange(len(scores)), class_ids]
    mask = confs >= 0.5
    if not np.any(mask):
        return []
    p = preds[mask]; ci = class_ids[mask]; c = confs[mask]
    cx, cy, bw, bh = p[:,0], p[:,1], p[:,2], p[:,3]
    x1=(cx-bw/2).astype(int); y1=(cy-bh/2).astype(int)
    x2=(cx+bw/2).astype(int); y2=(cy+bh/2).astype(int)
    boxes = np.stack([x1, y1, x2-x1, y2-y1], axis=1).tolist()
    return cv2.dnn.NMSBoxes(boxes, c.tolist(), 0.5, 0.45)

t_post_few = bench(postprocess_few, "[4a] Postprocess (few detections)")

high_boxes_output = np.random.rand(84 * 8400).astype(np.float32) * 2.0  # high conf → many boxes

def postprocess_many():
    preds = high_boxes_output.reshape(84, -1).T
    scores = preds[:, 4:]
    class_ids = np.argmax(scores, axis=1)
    confs = scores[np.arange(len(scores)), class_ids]
    mask = confs >= 0.5
    if not np.any(mask):
        return []
    p = preds[mask]; ci = class_ids[mask]; c = confs[mask]
    cx, cy, bw, bh = p[:,0], p[:,1], p[:,2], p[:,3]
    x1=(cx-bw/2).astype(int); y1=(cy-bh/2).astype(int)
    x2=(cx+bw/2).astype(int); y2=(cy+bh/2).astype(int)
    boxes = np.stack([x1, y1, x2-x1, y2-y1], axis=1).tolist()
    return cv2.dnn.NMSBoxes(boxes, c.tolist(), 0.5, 0.45)

t_post_many = bench(postprocess_many, "[4b] Postprocess (many detections)")

# 6. Preview encode (320x240 bgr8 tobytes — what cv2_to_imgmsg does)
def preview_encode():
    small = cv2.resize(frame_480p, (320, 240), interpolation=cv2.INTER_LINEAR)
    return small.tobytes()

t_pre_enc = bench(preview_encode, "[5] Preview resize+encode (320x240)")

# 7. JPEG encode for ZMQ stream
def jpeg_encode():
    ok, buf = cv2.imencode(".jpg", frame_480p, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return buf

t_jpeg = bench(jpeg_encode, "[6] JPEG encode for ZMQ (480p)")

# 8. cv_bridge full decode — what video_stream_node does for RAW camera topic
def decode_raw():
    raw = frame_480p.tobytes()
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(frame_480p.shape)
    return arr.copy()

t_decode_raw = bench(decode_raw, "[7] Raw camera imgmsg decode (full copy)")

print()
print("=" * 65)
print(" PIPELINE TOTALS")
print("=" * 65)

t_fixed = t_bridge + t_pre + t_trt + t_post_few + t_pre_enc
t_old   = t_bridge + t_pre_old + t_trt + t_post_few + t_pre_enc
print(f"  Theoretical FPS (fixed code):           {1000/t_fixed:6.1f} FPS  ({t_fixed:.1f}ms)")
print(f"  Theoretical FPS (old code with reset):  {1000/t_old:6.1f} FPS  ({t_old:.1f}ms)")
print()
print(f"  video_stream_node overhead (per frame):    {t_decode_raw + t_jpeg:.1f}ms")
print(f"    → subscribing raw camera = extra {t_decode_raw:.1f}ms decode + {t_jpeg:.1f}ms JPEG")
print()
print("  ⚠ NOTE: If actual FPS is 2-3, the bottleneck is NOT CPU pipeline.")
print("  Likely causes: GIL contention | camera FPS limited | DDS congestion")

ctx.pop()
