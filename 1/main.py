"""
Author: Interweave
Date: 2024/9/8
参考OpenCV文档: https://docs.opencv.org/4.x/index.html
"""


import cv2
import numpy as np

# 读取图像
image = cv2.imread('img.png')

# 转换为HSV色彩空间
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 定义蓝色的HSV范围
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])

# 定义橙色的BGR范围
lower_orange = np.array([0, 100, 200])
upper_orange = np.array([50, 170, 255])

# 蓝色区域 HSV
blue = cv2.inRange(hsv, lower_blue, upper_blue)

# 橙色区域 BGR
orange = cv2.inRange(image, lower_orange, upper_orange)

# 获取蓝色的轮廓
blues, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# 获取橙色的轮廓
oranges, _ = cv2.findContours(orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 遍历蓝色轮廓并画出矩形框
for i in blues:
    x, y, w, h = cv2.boundingRect(i)
    # 画出蓝色矩形框
    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    # 计算中心点坐标
    center_x = x + w // 2
    center_y = y + h // 2
    # 在图像上画出红色中心点
    cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)
    print(f"蓝色中心点坐标: ({center_x}, {center_y})")

# 遍历橙色轮廓并画出矩形框
for i in oranges:
    x, y, w, h = cv2.boundingRect(i)
    # 画出橙色矩形框
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 128, 255), 2)
    # 计算中心点坐标
    center_x = x + w // 2
    center_y = y + h // 2
    # 在图像上画出红色中心点
    cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)
    print(f"橙色中心点坐标: ({center_x}, {center_y})")


cv2.imshow('img', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
