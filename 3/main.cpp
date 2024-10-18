/*
Author: Interweave
Date: 2024/10/16
�ο�OpenCV�ĵ�: https://docs.opencv.org/4.x/index.html 
    POSIX ���׼ʾ��
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <fcntl.h>   
#include <unistd.h>
#include <termios.h>
#include <cstring>

// ����Lightbar�����ṹ��
struct Lightbar
{
    cv::Point2f center;  // �������ĵ�
    float width;         // �������
    float height;        // �����߶�
};

// ����Armorװ�װ�ṹ��
struct Armor
{
    cv::Point2f center;       // װ�װ����ĵ�
    Lightbar lightbar1;       // ����1
    Lightbar lightbar2;       // ����2
};

// ����armor��ѡװ�װ�
void drawRotatedRect(cv::Mat& image, const Armor& armor)
{
    // �������֮��ĽǶ�
    float angle = std::atan2(armor.lightbar2.center.y - armor.lightbar1.center.y, armor.lightbar2.center.x - armor.lightbar1.center.x) * 180.0 / CV_PI;

    // ���þ��εĿ�Ⱥ͸߶�
    float width = cv::norm(armor.lightbar2.center - armor.lightbar1.center);
    float height = (armor.lightbar1.height + armor.lightbar2.height) / 2 *2.1;

    // ������ת����
    cv::RotatedRect rotatedRect(armor.center, cv::Size2f(width, height), angle);

    // ��ȡ���ε��ĸ�����
    cv::Point2f vertices[4];
    rotatedRect.points(vertices);

    // ���ƾ���
    for (int i = 0; i < 4; ++i)
        cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
}


// ����Ŀ���pitch��yaw
cv::Point2f getPitchYaw(const cv::Point2f& center_point) {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        2866.09828, 0, 339.173314,
        0, 469.996560, 210.127011,
        0, 0, 1);

    float focal_length = 2866.09828;  // ����

    float z = focal_length;  
    float x = (center_point.x - camera_matrix.at<double>(0, 2)) * focal_length / camera_matrix.at<double>(0, 0);
    float y = (center_point.y - camera_matrix.at<double>(1, 2)) * focal_length / camera_matrix.at<double>(1, 1);

    // ����pitch��yaw
    float pitch = atan2(y, z) * 180.0 / CV_PI;  
    float yaw = atan2(x, z) * 180.0 / CV_PI;  

    return cv::Point2f(pitch, yaw);
}


int main()
{
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
    
    
    // ��ɫHSV
    cv::Scalar lowerWhite(0, 0, 200);
    cv::Scalar upperWhite(180, 60, 255);

    // ����Ƶ�ļ�
    cv::VideoCapture cap("1.mp4");
    // cv::VideoCapture cap("2.mp4");  
    if (!cap.isOpened())
    {
        std::cerr << "��ȡʧ��" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        // ��ȡ֡�Ŀ��
        int width = frame.cols;
        // ��ȡ֡�ĸ߶�
        int height = frame.rows;

        cv::Point2f frameCenter(width / 2.0f, height / 2.0f);

        // ��ȡ����
        cv::Mat hsv, binary, gray;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lowerWhite, upperWhite, hsv);

        // ת��Ϊ�Ҷ�ͼ
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ��ȡ������ɫ����
        cv::threshold(gray, binary, 180, 255, cv::THRESH_BINARY);

        // �ϲ�
        binary = hsv | binary;

        // ��������
        // cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::GaussianBlur(binary, binary, cv::Size(7, 7), 1);
        cv::blur(binary, binary, cv::Size(5, 5));

        // ��ȡ����
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // ��������ṹ���б�
        std::vector<Lightbar> lightbars;

        // ɸѡ����
        for (const auto& contour : contours)
        {
            float area = cv::contourArea(contour);
            cv::Rect boundingBox = cv::boundingRect(contour);
            // ���������
            double proportion = static_cast<double>(boundingBox.width) /boundingBox.height;
            
            if (area > 5 && proportion > 0.1 && proportion < 0.88)
            {
                // ������������
                Lightbar lightbar;
                lightbar.center = (boundingBox.tl() + boundingBox.br()) * 0.5;
                lightbar.width = static_cast<float>(boundingBox.width);
                lightbar.height = static_cast<float>(boundingBox.height);
                
                // �洢����
                lightbars.push_back(lightbar);
            }
        }

        // ����������״̬
        std::vector<bool> paired(lightbars.size(), false);

        // ����װ�װ�
        std::vector<Armor> armors;

        // ���е�����ԣ�����װ�װ�
        for (size_t i = 0; i < lightbars.size(); i++)
        {
            if (paired[i]) continue;

            for (size_t j = i + 1; j < lightbars.size(); j++)
            {
                if (paired[j]) continue;

                // �����������
                float lightDistance = cv::norm(lightbars[i].center - lightbars[j].center);
                
                //���Ե������Ⱥ�װ�װ峤�ȱ�����Χ
                //std::cout << lightbars[i].height / lightDistance << std::endl;

                float proportion = lightbars[i].height / lightDistance; 


                if (proportion > 0.35 && proportion < 0.68)
                {
                    // ����һ��װ�װ����
                    Armor armor;
                    armor.center = (lightbars[i].center + lightbars[j].center) * 0.5;
                    armor.lightbar1 = lightbars[i];
                    armor.lightbar2 = lightbars[j];

                    // ����װ�װ�����
                    armors.push_back(armor);

                    paired[i] = true;
                    paired[j] = true;

                    break;
                }
            }
        }

        // �ҳ��������������װ�װ�
        if (!armors.empty())
        {
            // �ȼ����һ��װ�װ�Ϊ��ӽ����ĵ�
            Armor closestArmor = armors[0];
            // ������С����,��ʼֵΪ��һ��װ�װ������
            float minDistance = cv::norm(closestArmor.center - frameCenter);

            // �������д��ڵ�װ�װ�
            for (const auto& armor : armors)
            {
                // ͨ������������
                float distance = cv::norm(armor.center - frameCenter);
                if (distance < minDistance)
                {
                    closestArmor = armor;
                    minDistance = distance;
                }
            }

            // ��ѡ�������������װ�װ�
            drawRotatedRect(frame, closestArmor);

            // ����װ�װ�����
            cv::circle(frame, closestArmor.center, 5, cv::Scalar(255, 0, 255), -1);

            //���pitch��yawֵ
            cv::Point2f pitch_yaw = getPitchYaw(closestArmor.center);
            
            // ��������
            std::string data = "Pitch:" + std::to_string(pitch_yaw.x) + ";Yaw:" + std::to_string(pitch_yaw.y) + "\n";
            const char* message = data.c_str() ;
    	    write(serialPort, message, strlen(message));

    	    // ��������
    	    char buffer[256];
    	    int bytesRead = read(serialPort, buffer, sizeof(buffer));
    	    if (bytesRead > 0) {
      	        std::cout << std::string(buffer, bytesRead) << std::endl;
    	    }
    	    
            // ������
            //std::cout << "Pitch: " << pitch_yaw.x << " Yaw: " << pitch_yaw.y << std::endl;
        }


        /*
        // ����(���paired)
        for (size_t i = 0; i < paired.size(); ++i)
        {
            std::cout << "paired[" << i << "] = " << paired[i] << std::endl;
        }
        */

        /*
        // ����(���armors)
        for (size_t i = 0; i < armors.size(); ++i)
        {
            std::cout << "armer[" << i << "].center = " << armors[i].center << std::endl;
            std::cout << "armer[" << i << "].lightbar1 = " << armors[i].lightbar1.center << std::endl;
            std::cout << "armer[" << i << "].lightbar2 = " << armors[i].lightbar2.center << std::endl;
        }
        */

        /*
        // ����(���lightbars)
        for (size_t i = 0; i < lightbars.size(); ++i)
        {
            std::cout << "lightbars[" << i << "] = " << lightbars[i].center << std::endl;
        }
        */

        // ��ʾ���ͼ��
        cv::imshow("main", frame);

        // ��ʾ����ͼ��
        // cv::imshow("binary", binary);

        if (cv::waitKey(30) >= 0)
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
