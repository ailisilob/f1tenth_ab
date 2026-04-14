# Lab 8: Vision Lab

## The x, y distance of the unknown cones?
x_car = 275.1 mm (27.5 cm)
y_car = 291.0 mm (29.1 cm)

Measured using camera intrinsic matrix K obtained from checkerboard calibration
(reprojection error: 0.6439 pixels), and camera mounting height H = 1294.7 mm
calculated from the reference image cone_x40cm.png.

## Lane Detection Result Image
See lane_result.png - Yellow lane markers detected using HSV color filtering
(H: 15-40, S: 40-255, V: 40-255) with morphological operations and contour
detection. Green contours are drawn around detected yellow lane markers.

## Integrated Object Detection + Distance Calculation Result Image
See integrated_result.png - Pipeline captures 960x540 frames at ~60fps using
RealSense camera, runs yellow lane detection and F110 car object detection
simultaneously. Distance to detected object is calculated using the bottom
center point of the bounding box projected to car frame coordinates.

## Neural Network Training & Testing Loss Plot
See loss plot - Trained custom YOLO-style network for F110 car detection.
Network architecture: 9 conv layers with BatchNorm, trained for 150 epochs
with batch_size=16, lr=1e-3, Adam optimizer.
Final train loss: ~500, Final validation loss: ~500.

## Is FP16 faster? Why?
FP16 inference time (ms): 0.76 ms (1312.0 FPS)

FP32 inference time (ms): 1.46 ms (685.9 FPS)

Speedup: 1.91x faster with FP16.

Yes, FP16 is faster. FP16 (half precision) uses 16-bit floating point numbers
instead of 32-bit, which means each operation requires half the memory bandwidth
and the GPU can process twice as many values per clock cycle using SIMD
instructions. The Jetson's GPU has dedicated FP16 tensor cores that further
accelerate half-precision computation, resulting in ~2x speedup with minimal
accuracy loss for inference tasks.