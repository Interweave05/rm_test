/*
Author: Interweave
Date: 2024/10/16
�ο�OpenCV�ĵ�: https://docs.opencv.org/4.x/index.html 
    POSIX ���׼ʾ��
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
    // �򿪴���
    int serialPort = open("/dev/ttyUSB0", O_RDWR);
    if (serialPort < 0) {
        std::cerr << "�޷��򿪴���" << std::endl;
        return 1;
    }
    struct termios tty;    
    // �����������������Ϊ 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    // ���� 8 ����λ����У��λ��1 ֹͣλ��8N1 ģʽ��
    tty.c_cflag &= ~PARENB; // ��У��
    tty.c_cflag &= ~CSTOPB; // 1 ֹͣλ
    tty.c_cflag &= ~CSIZE;  // �����ǰ����λ����
    tty.c_cflag |= CS8;     // ���� 8 ����λ
    tty.c_cflag &= ~CRTSCTS; // ���ô��ڲ�ʹ��Ӳ��������
    tty.c_cflag |= (CLOCAL | CREAD); // ���ý������ͱ���ģʽ
    // Ӧ������
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "�޷����ô�������" << std::endl;
        close(serialPort);
        return 1;
    }

    // ��ȡ��Ƶ
    VideoCapture cap("1.mp4");

    // ��ɫ��HSV��Χ
    Scalar lower_red1(0, 130, 70);
    Scalar upper_red1(9, 255, 255);
    Scalar lower_red2(170, 120, 70);
    Scalar upper_red2(180, 255, 255);

    // ���������Χ
    double min_ratio = 0.85;
    double max_ratio = 1.28;

    while (true) {
        Point top_left(-1, -1); // ��ʼ�������Ͻǵ�����

        Mat frame, hsv, mask1, mask2, red;
        bool ret = cap.read(frame);
        if (!ret) {
            break;
        }

        // ת��ΪHSV��ɫ�ռ�
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // ���ɺ�ɫ���� (��ɫ��HSV�е���������)
        inRange(hsv, lower_red1, upper_red1, mask1);
        inRange(hsv, lower_red2, upper_red2, mask2);

        red = mask1 + mask2;

        // ��˹�˲�
        GaussianBlur(red, red, Size(5, 5), 1);

        // ��ֵ�˲�
        blur(red, red, Size(3, 3));

        // ��ֵ�˲�
        medianBlur(red, red, 3);

        // ������
        morphologyEx(red, red, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(26, 26)));

        // ������
        morphologyEx(red, red, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(23, 23)));

        // ��ʴ
        // erode(red, red, getStructuringElement(MORPH_RECT, Size(3, 3)));

        // �ҵ���ɫ���������
        vector<vector<Point>> contours;
        findContours(red, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (auto& contour : contours) {
            double area = contourArea(contour);
            Rect boundRect = boundingRect(contour);
            double aspect_ratio = (double)boundRect.width / boundRect.height;

            if (area > 300 && area < 5600 && aspect_ratio >= min_ratio && aspect_ratio <= max_ratio) {
                // ���������ͱ߽��
                rectangle(frame, boundRect.tl(), boundRect.br(), Scalar(0, 255, 0), 2);
		// ��������������
		Point center;
		center.x = boundRect.x + boundRect.width / 2;
		center.y = boundRect.y + boundRect.height / 2;
                // �ҵ������Ͻǵĺ�ɫ����
                if (top_left.x == -1 || (boundRect.x < top_left.x && boundRect.y < top_left.y)) {
                    top_left = center;

                }
                
                // ��������
            	std::string data = "(" + std::to_string(top_left.x) + ", " + std::to_string(top_left.y) + ")\n";

            	const char* message = data.c_str() ;
    	    	write(serialPort, message, strlen(message));
    	    	// ��������
    	    	char buffer[256];
    	    	int bytesRead = read(serialPort, buffer, sizeof(buffer));
    	    	if (bytesRead > 0) {
      	        std::cout << std::string(buffer, bytesRead) << std::endl;    	    	
    	    	}
    	    	
                
            }
        }

        // ��ʾ���
        imshow("image", frame);
        // imshow("test", red);

        // ���� 'q' ���˳�
        if (waitKey(30) == 'q') {
            break;
        }
    }

    cap.release();
    destroyAllWindows();

    return 0;
}
