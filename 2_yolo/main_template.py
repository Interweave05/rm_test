"""
Author: Interweave
Date: 2024/9/16
参考OpenCV文档: https://docs.opencv.org/4.x/index.html
"""

import cv2
import numpy as np

# 读取视频
cap = cv2.VideoCapture('1.mp4')

# 红色的HSV范围
lower_red1 = np.array([0, 130, 70])
upper_red1 = np.array([9, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

# 长宽比例范围
min_ratio = 0.85
max_ratio = 1.28

while True:
    # 初始化最左上角的坐标
    top_left = None

    ret, image = cap.read()
    if not ret:
        break

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 生成红色掩码 (红色在HSV中的两个区间)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    red = mask1 + mask2

    # 膨胀
    # red = cv2.dilate(red, np.ones((3, 3), np.uint8), iterations=1)

    # 开运算
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((26, 26), np.uint8))

    # 闭运算
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((23, 23), np.uint8))

    # 腐蚀
    red = cv2.erode(red, np.ones((3, 3), np.uint8), iterations=1)

    # 找到红色区域的轮廓
    reds, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for i in reds:
        area = cv2.contourArea(i)
        x, y, w, h = cv2.boundingRect(i)

        aspect_ratio = w / h

        # print(aspect_ratio)

        if 300 < area < 5600 and min_ratio <= aspect_ratio <= max_ratio:
            # 绘制轮廓和边界框
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 找到最左上角的红色方块
            if top_left is None or (x < top_left[0] and y < top_left[1]):
                top_left = (x, y)
                centerred = ((x+w)/2, (y+h)/2)
    # 显示结果
    cv2.imshow('image', image)
    # 测试用
    cv2.imshow('test', red)

    # 按下 'q' 键退出
    if cv2.waitKey(1000 // 30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
