import sys
sys.path.insert(0, '/media/team4/bbc64d1c-24dc-4aa3-8c44-8dfd6c37ff3b2/usr/lib/python3.10/dist-packages/tensorrt')
sys.path = [p for p in sys.path if 'bbc64d1c' not in p or 'tensorrt' in p]

import tensorrt as trt
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
import cv2
import time

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

# Constants
INPUT_DIM   = [180, 320]
FINAL_DIM   = [5, 10]
ANCHOR_SIZE = [INPUT_DIM[0] / FINAL_DIM[0], INPUT_DIM[1] / FINAL_DIM[1]]

def load_engine(engine_path):
    with open(engine_path, 'rb') as f:
        runtime = trt.Runtime(TRT_LOGGER)
        return runtime.deserialize_cuda_engine(f.read())

def preprocess(image):
    img = cv2.resize(image, (INPUT_DIM[1], INPUT_DIM[0]))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.ascontiguousarray(img[np.newaxis])  # (1, 3, 180, 320)
    return img

def grid_cell(col, row):
    return np.array([col * ANCHOR_SIZE[1],
                     row * ANCHOR_SIZE[0],
                     col * ANCHOR_SIZE[1] + ANCHOR_SIZE[1],
                     row * ANCHOR_SIZE[0] + ANCHOR_SIZE[0]])

def postprocess(output, threshold=0.1):
    # Use numpy only, no pytorch
    result = output.reshape(5, FINAL_DIM[0], FINAL_DIM[1])
    best_conf = -1
    best_box  = None
    print(f"Max confidence: {result[0].max():.4f}")
    for row in range(FINAL_DIM[0]):
        for col in range(FINAL_DIM[1]):
            conf = result[0, row, col]
            if conf >= threshold and conf > best_conf:
                best_conf = conf
                grid = grid_cell(col, row)
                cx = grid[0] + ANCHOR_SIZE[1]/2 + result[1, row, col]
                cy = grid[1] + ANCHOR_SIZE[0]/2 + result[2, row, col]
                w  = result[3, row, col] * INPUT_DIM[1]
                h  = result[4, row, col] * INPUT_DIM[0]
                best_box = (cx, cy, w, h, conf)
    return best_box

def detect(image, engine):
    context = engine.create_execution_context()
    inp = preprocess(image)
    out = np.empty((1, 5, FINAL_DIM[0], FINAL_DIM[1]), dtype=np.float32)
    context.set_input_shape('input', inp.shape)
    d_input  = cuda.mem_alloc(inp.nbytes)
    d_output = cuda.mem_alloc(out.nbytes)
    cuda.memcpy_htod(d_input, inp)
    context.execute_v2([int(d_input), int(d_output)])
    cuda.memcpy_dtoh(out, d_output)
    return postprocess(out[0])

def benchmark(engine_path, image, runs=100):
    engine  = load_engine(engine_path)
    context = engine.create_execution_context()
    inp = preprocess(image)
    out = np.empty((1, 5, FINAL_DIM[0], FINAL_DIM[1]), dtype=np.float32)
    context.set_input_shape('input', inp.shape)
    d_input  = cuda.mem_alloc(inp.nbytes)
    d_output = cuda.mem_alloc(out.nbytes)
    cuda.memcpy_htod(d_input, inp)

    # Warmup
    for _ in range(10):
        context.execute_v2([int(d_input), int(d_output)])

    # Benchmark
    t0 = time.time()
    for _ in range(runs):
        context.execute_v2([int(d_input), int(d_output)])
    avg_ms = (time.time() - t0) / runs * 1000
    print(f"{engine_path}: avg = {avg_ms:.2f} ms ({1000/avg_ms:.1f} FPS)")
    return avg_ms

if __name__ == '__main__':
    test_image = np.random.randint(0, 255, (540, 960, 3), dtype=np.uint8)

    print("=== Benchmarking FP32 engine ===")
    fp32_ms = benchmark('model_fp32.trt', test_image)

    print("\n=== Benchmarking FP16 engine ===")
    fp16_ms = benchmark('model_fp16.trt', test_image)

    print(f"\nSpeedup FP16 vs FP32: {fp32_ms/fp16_ms:.2f}x")

    print("\n=== Testing detection with FP32 engine ===")
    engine = load_engine('model_fp32.trt')
    box = detect(test_image, engine)
    print(f"Detection result: {box}")