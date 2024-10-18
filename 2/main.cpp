/*
Author: Interweave
Date: 2024/10/16
参考OpenCV文档: https://docs.opencv.org/4.x/index.html 
    POSIX 库标准示例
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>   
#include <unistd.h>
#include <termios.h>
#include <cstring>

using namespace cv;
using namespace std;

int main() {
    // 打开串口
    int serialPort = open("/dev/ttyUSB0", O_RDWR);
    if (serialPort < 0) {
        std::cerr << "无法打开串口" << std::endl;
        return 1;
    }
    struct termios tty;    
    // 设置输入输出波特率为 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    // 配置 8 数据位，无校验位，1 停止位（8N1 模式）
    tty.c_cflag &= ~PARENB; // 无校验
    tty.c_cflag &= ~CSTOPB; // 1 停止位
    tty.c_cflag &= ~CSIZE;  // 清除当前数据位设置
    tty.c_cflag |= CS8;     // 设置 8 数据位
    tty.c_cflag &= ~CRTSCTS; // 设置串口不使用硬件流控制
    tty.c_cflag |= (CLOCAL | CREAD); // 启用接收器和本地模式
    // 应用设置
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "无法设置串口属性" << std::endl;
        close(serialPort);
        return 1;
    }

    // 读取视频
    VideoCapture cap("1.mp4");

    // 红色的HSV范围
    Scalar lower_red1(0, 130, 70);
    Scalar upper_red1(9, 255, 255);
    Scalar lower_red2(170, 120, 70);
    Scalar upper_red2(180, 255, 255);

    // 长宽比例范围
    double min_ratio = 0.85;
    double max_ratio = 1.28;

    while (true) {
        Point top_left(-1, -1); // 初始化最左上角的坐标

        Mat frame, hsv, mask1, mask2, red;
        bool ret = cap.read(frame);
        if (!ret) {
            break;
        }

        // 转换为HSV颜色空间
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // 生成红色掩码 (红色在HSV中的两个区间)
        inRange(hsv, lower_red1, upper_red1, mask1);
        inRange(hsv, lower_red2, upper_red2, mask2);

        red = mask1 + mask2;

        // 高斯滤波
        GaussianBlur(red, red, Size(5, 5), 1);

        // 均值滤波
        blur(red, red, Size(3, 3));

        // 中值滤波
        medianBlur(red, red, 3);

        // 开运算
        morphologyEx(red, red, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(26, 26)));

        // 闭运算
        morphologyEx(red, red, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(23, 23)));

        // 腐蚀
        // erode(red, red, getStructuringElement(MORPH_RECT, Size(3, 3)));

        // 找到红色区域的轮廓
        vector<vector<Point>> contours;
        findContours(red, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (auto& contour : contours) {
            double area = contourArea(contour);
            Rect boundRect = boundingRect(contour);
            double aspect_ratio = (double)boundRect.width / boundRect.height;

            if (area > 300 && area < 5600 && aspect_ratio >= min_ratio && aspect_ratio <= max_ratio) {
                // 绘制轮廓和边界框
                rectangle(frame, boundRect.tl(), boundRect.br(), Scalar(0, 255, 0), 2);
		// 计算其中心坐标
		Point center;
		center.x = boundRect.x + boundRect.width / 2;
		center.y = boundRect.y + boundRect.height / 2;
                // 找到最左上角的红色方块
                if (top_left.x == -1 || (boundRect.x < top_left.x && boundRect.y < top_left.y)) {
                    top_left = center;

                }
                
                // 发送数据
            	std::string data = "(" + std::to_string(top_left.x) + ", " + std::to_string(top_left.y) + ")\n";

            	const char* message = data.c_str() ;
    	    	write(serialPort, message, strlen(message));
    	    	// 接收数据
    	    	char buffer[256];
    	    	int bytesRead = read(serialPort, buffer, sizeof(buffer));
    	    	if (bytesRead > 0) {
      	        std::cout << std::string(buffer, bytesRead) << std::endl;    	    	
    	    	}
    	    	
                
            }
        }

        // 显示结果
        imshow("image", frame);
        // imshow("test", red);

        // 按下 'q' 键退出
        if (waitKey(30) == 'q') {
            break;
        }
    }

    cap.release();
    destroyAllWindows();

    return 0;
}
