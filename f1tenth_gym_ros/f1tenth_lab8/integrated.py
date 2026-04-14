import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import torch.nn as nn

# ───────── Network definition (must match training) ─────────
class F110_YOLO(torch.nn.Module):
    def __init__(self):
        super(F110_YOLO, self).__init__()
        self.conv1      = nn.Conv2d(3,   16,  3, padding=1)
        self.batchnorm1 = nn.BatchNorm2d(16)
        self.conv2      = nn.Conv2d(16,  32,  3, padding=1)
        self.batchnorm2 = nn.BatchNorm2d(32)
        self.conv3      = nn.Conv2d(32,  64,  3, padding=1)
        self.batchnorm3 = nn.BatchNorm2d(64)
        self.conv4      = nn.Conv2d(64,  64,  3, padding=1)
        self.batchnorm4 = nn.BatchNorm2d(64)
        self.conv5      = nn.Conv2d(64,  128, 3, padding=1)
        self.batchnorm5 = nn.BatchNorm2d(128)
        self.conv6      = nn.Conv2d(128, 128, 3, padding=1)
        self.batchnorm6 = nn.BatchNorm2d(128)
        self.conv7      = nn.Conv2d(128, 64,  3, padding=1)
        self.batchnorm7 = nn.BatchNorm2d(64)
        self.conv8      = nn.Conv2d(64,  32,  3, padding=1)
        self.batchnorm8 = nn.BatchNorm2d(32)
        self.conv9      = nn.Conv2d(32,  5,   1)
        self.pool       = nn.MaxPool2d(2, 2)

    def forward(self, x):
        x = self.pool(torch.relu(self.batchnorm1(self.conv1(x))))
        x = self.pool(torch.relu(self.batchnorm2(self.conv2(x))))
        x = torch.relu(self.batchnorm3(self.conv3(x)))
        x = self.pool(torch.relu(self.batchnorm4(self.conv4(x))))
        x = torch.relu(self.batchnorm5(self.conv5(x)))
        x = self.pool(torch.relu(self.batchnorm6(self.conv6(x))))
        x = torch.relu(self.batchnorm7(self.conv7(x)))
        x = nn.functional.avg_pool2d(
            torch.relu(self.batchnorm8(self.conv8(x))),
            kernel_size=(2, 2), stride=(2, 2))
        x = self.conv9(x)
        x = torch.cat([x[:, 0:3, :, :], torch.sigmoid(x[:, 3:5, :, :])], dim=1)
        return x

# ───────── Constants ─────────
INPUT_DIM   = [180, 320]   # detection network input size
FINAL_DIM   = [5, 10]      # network output grid size
ANCHOR_SIZE = [INPUT_DIM[0] / FINAL_DIM[0], INPUT_DIM[1] / FINAL_DIM[1]]

# Camera intrinsics from calibration
K = np.array([[694.71580861,   0.,         449.37451805],
              [  0.,         695.5498873,  258.64744541],
              [  0.,           0.,           1.        ]])
H = 1294.7  # camera mounting height in mm

# ───────── Helper functions ─────────
def grid_cell(col, row):
    return np.array([col * ANCHOR_SIZE[1],
                     row * ANCHOR_SIZE[0],
                     col * ANCHOR_SIZE[1] + ANCHOR_SIZE[1],
                     row * ANCHOR_SIZE[0] + ANCHOR_SIZE[0]])

def bbox_convert(cx, cy, w, h):
    return [cx - w/2, cy - h/2, cx + w/2, cy + h/2]

def label_to_box(result, threshold=0.05):
    best_conf = -1
    best_box  = None
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

def pixel_to_car(u, v):
    fx, fy = K[0,0], K[1,1]
    cx, cy = K[0,2], K[1,2]
    x_car = (u - cx) / fx * H
    y_car = (v - cy) / fy * H
    return x_car, y_car

def detect_lane(image):
    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15,  40,  40])
    upper_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    result = image.copy()
    for cnt in contours:
        if cv2.contourArea(cnt) > 300:
            cv2.drawContours(result, [cnt], -1, (0, 255, 0), 3)
    return result

def preprocess(frame):
    # Resize to network input, normalize, convert to tensor
    img = cv2.resize(frame, (INPUT_DIM[1], INPUT_DIM[0]))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    return torch.from_numpy(img).unsqueeze(0).float()

# ───────── Main ─────────
def main():
    # Load model
    device = torch.device('cpu')
    model  = F110_YOLO().to(device)
    model.load_state_dict(torch.load('model_final.pt', map_location=device))
    model.eval()
    print("Model loaded successfully")

    # Open camera
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60)
    pipeline.start(config)
    print("Camera started")

    try:
        while True:
            # 1. Get frame and resize to 960x540
            frames      = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            frame = cv2.resize(frame, (960, 540))

            # 2. Lane detection on full resolution frame
            result_img = detect_lane(frame)

            # 3. Object detection
            with torch.no_grad():
                inp    = preprocess(frame).to(device)
                output = model(inp)
                output = output.squeeze(0).numpy()

            # 4. Get best bounding box
            box = label_to_box(output, threshold=0.4)

            if box is not None:
                cx, cy, w, h, conf = box
                # Scale from detection resolution to 960x540
                scale_x = 960 / INPUT_DIM[1]
                scale_y = 540 / INPUT_DIM[0]
                cx_img  = cx * scale_x
                cy_img  = cy * scale_y
                w_img   = w  * scale_x
                h_img   = h  * scale_y

                # Bottom center point of bounding box for distance measurement
                bottom_u = cx_img
                bottom_v = cy_img + h_img / 2
                x_car, y_car = pixel_to_car(bottom_u, bottom_v)

                # Draw bounding box
                x1 = int(cx_img - w_img / 2)
                y1 = int(cy_img - h_img / 2)
                x2 = int(cx_img + w_img / 2)
                y2 = int(cy_img + h_img / 2)
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(result_img, (int(bottom_u), int(bottom_v)), 5, (255, 0, 0), -1)
                cv2.putText(result_img,
                            f"x:{x_car:.0f}mm y:{y_car:.0f}mm conf:{conf:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 5. Save output frame for verification
            cv2.imwrite('integrated_result.png', result_img)
            print(f"Frame processed, box={'detected' if box else 'none'}")
            break  # Remove this line to run continuously

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        pipeline.stop()
        print("Camera closed")

if __name__ == '__main__':
    main()