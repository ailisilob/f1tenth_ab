import pyrealsense2 as rs
import numpy as np
import cv2
import time

def open_camera(width=424, height=240, fps=60):
    # 424x240 @ 60fps is the only configuration supporting 60Hz
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)
    print(f"Camera opened: {width}x{height} @ {fps}fps")
    return pipeline

def get_frame(pipeline, target_w=960, target_h=540):
    # Capture at 424x240 then resize to 960x540
    frames = pipeline.wait_for_frames(timeout_ms=5000)
    color_frame = frames.get_color_frame()
    if not color_frame:
        return False, None
    frame = np.asanyarray(color_frame.get_data())
    frame = cv2.resize(frame, (target_w, target_h))
    return True, frame

if __name__ == '__main__':
    pipeline = open_camera()

    prev  = time.time()
    count = 0

    try:
        while True:
            ret, frame = get_frame(pipeline)
            if not ret:
                print("Failed to get frame")
                continue

            count += 1
            now = time.time()
            if now - prev >= 1.0:
                print(f"Measured FPS: {count/(now-prev):.1f}  shape: {frame.shape}")
                count = 0
                prev  = now

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        pipeline.stop()
        print("Camera closed")