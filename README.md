https://github.com/Interweave05/rm_test
# TGU视觉组招新试题
## 1 基础题-魔方色块识别
使用HSV识别出蓝色区域，BGR(RGB)识别出橙色区域。获取轮廓，计算矩形框中心点，并输出结果
## 2 进阶题-识别视频魔方红色色块
使用红色的两段HSV范围，识别出当前帧所有的红色区域。再使用高斯滤波、中值滤波、开闭运算等方法，尽可能去除图像干扰。
使用面积和长宽比值，找到符合要求的红色区域，即视为魔方色块，计算其中心点，使用串口发送数据。
## 3 附加题-装甲板识别及其Yaw、Pitch轴计算
为了程序更易于编写，定义了灯条与装甲板结构体。以及定义一个使用Armor结构体参数绘制旋转矩形的函数和一个计算Pitch和Yaw角度的函数。
同理，使用白色HSV和二值图像，分别获取白色区域图像，再使用"|"位运算进行合并,获得更为完整精确清晰的二值图像。
再经过图像降噪处理后，使用长宽比以及白色区域面积的方法筛选灯条，将灯条参数存入LightBar结构体列表中。
然后以结构体列表中的灯条两两比较，使用灯条长度和装甲板长度比例判断其是否为同一装甲板的灯条，将符合条件的装甲板存入装甲板结构体列表中
最后遍历装甲板列表，计算距离中心最小距离的装甲板，将其框选出，并在串口输出其Yaw和Pitch角度
## 2_yolo YOLO版本的进阶题做法
使用YOLOv5模型先对图像中魔方进行预测，获得其大致位置，再使用ROI对魔方进行处理，后同2
## ESP32-S3 串口设备程序
本项目中的串口设备使用ESP32-S3，其程序为将接收到的串口数据再次发送回原设备
