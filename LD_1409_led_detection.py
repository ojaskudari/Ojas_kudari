import cv2
import numpy as np

image = cv2.imread('led.jpg', 1)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, ksize=(11, 11), sigmaX=cv2.BORDER_DEFAULT)

# Threshold to create a binary mask of white regions
_, mask = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

# Find contours in the binary mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

centroid_list = []
area_list = []

for contour in contours:
    # Calculate the area of the contour
    area = cv2.contourArea(contour)

    # Calculate the centroid of the contour
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    centroid_list.append((cx, cy))
    area_list.append(area)

    
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)


cv2.imwrite("led_detection_results.png", image)


with open("led_detection_results.txt", "w") as file:
    file.write(f"No. of LEDs detected: {len(contours)}\n")
    for i, centroid in enumerate(centroid_list):
        area = area_list[i]
        file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")

cv2.imshow("image", blur)
cv2.waitKey(0)
cv2.destroyAllWindows()