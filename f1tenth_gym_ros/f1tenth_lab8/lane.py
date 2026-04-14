import cv2
import numpy as np

def detect_lane(image):
    # Convert to HSV for color filtering
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # --- Yellow lane markers only ---
    lower_yellow = np.array([18,  50, 50])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Morphological operations to clean up noise
    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    result = image.copy()
    for cnt in contours:
        if cv2.contourArea(cnt) > 300:
            cv2.drawContours(result, [cnt], -1, (0, 255, 0), 3)

    return result

if __name__ == '__main__':
    img = cv2.imread('resource/lane.png')
    if img is None:
        print("Failed to load image")
        exit()

    print(f"Image shape: {img.shape}")
    result = detect_lane(img)
    cv2.imwrite('lane_result.png', result)
    print("Result saved to lane_result.png")