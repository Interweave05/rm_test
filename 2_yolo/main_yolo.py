"""
Author: Interweave
Date: 2024/10/5
参考OpenCV文档: https://docs.opencv.org/4.x/index.html
"""

import cv2
import torch
import numpy as np
import serial

min_x = 99999999
min_y = 99999999
centerred = (0,0)
# 设置串口参数
port = 'COM5'
baudrate = 115200
timeout = 1

# 打开串口
ser = serial.Serial(port, baudrate, timeout=timeout)

if ser.is_open:
    print(f"{port} 已打开")

# 加载YOLOv5模型
model = torch.hub.load('ultralytics/yolov5', 'custom', path='cube.pt', device='cpu')
# model = torch.hub.load('ultralytics/yolov5', 'custom', path='cube.pt', device=0)

# 设置置信度阈值
confidence_threshold = 0.8

# 定义ROI冗余量
delta = 13

# 红色的HSV范围
lower_red1 = np.array([0, 120, 60])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([165, 110, 60])
upper_red2 = np.array([180, 255, 255])

# 打开视频文件
cap = cv2.VideoCapture('1.mp4')
if not cap.isOpened():
    print("Error")
    exit()

while True:
    ret, frame = cap.read()  # 读取视频帧
    if not ret:
        break

    # 对视频帧进行图像预测,并获取预测结果,取列表的第一个结果
    results = model(frame)
    detections = results.xyxy[0]  # (x1, y1, x2, y2, confidence, class)
    detections = detections.tolist()
    detections = detections[0]

    # 测试
    # print(detections)
    # print(detections)

    # 分别获取所需要的值并转为整数,并适当扩大范围，防止截取不全
    x1, y1, x2, y2, conf, cls = map(int, detections)
    x1, y1, x2, y2 = x1 - delta, y1 - delta, x2 + delta, y2 + delta

    # 测试
    # print(x1, y1, x2, y2, conf, cls)

    # 利用yolo获取的物体位置,截取ROI区域
    roi = frame[y1:y2, x1:x2]
    # cv2.imshow('ROI', roi)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # cv2.imshow('HSV', hsv)

    # 生成红色掩码 (红色在HSV中的两个区间)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red = mask1 + mask2

    # 开运算
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((26, 26), np.uint8))

    # 测试
    # cv2.imshow('red', red)

    # 找到红色区域的轮廓
    reds, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_x = 999999
    min_y = 999999
    for i in reds:
        area = cv2.contourArea(i)
        x, y, w, h = cv2.boundingRect(i)
        x, y = x+x1, y+y1
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 寻找左上角的红色方块
        if x<=min_x and y<=min_y:
            min_x = x
            min_y = y
            min_w = w
            min_h = h
            centerred = ((x+w)/2, (y+h)/2)
        ser.write((str(centerred) + '\n').encode('utf-8'))
        data = ser.readline().decode("UTF-8").strip()  # 读取一行并去掉结尾的换行符
        print(f"{data}")
    cv2.imshow('frame', frame)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break