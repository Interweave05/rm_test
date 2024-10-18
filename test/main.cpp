#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath> 

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
   
    float width = cv::norm(armor.lightbar2.center - armor.lightbar1.center);
    float height = ((armor.lightbar1.height + armor.lightbar2.height) / 2) * 2.2;

    // ������ת����
    cv::RotatedRect rotatedRect(armor.center, cv::Size2f(width, height), angle);

    // ��ȡ���ε��ĸ�����
    cv::Point2f vertices[4];
    rotatedRect.points(vertices);

    // ���ƾ���
    for (int i = 0; i < 4; ++i)
        cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
}

int main()
{
    // ��ɫHSV
    cv::Scalar lowerWhite(0, 0, 200);
    cv::Scalar upperWhite(180, 60, 255);

    // ����Ƶ�ļ�
    cv::VideoCapture cap("1.mp4"); 
    if (!cap.isOpened())
    {
        std::cerr << "��ȡʧ��" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true)
    {
        // ����Ƶ�ж�ȡÿһ֡
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
        cv::threshold(gray, binary, 230, 255, cv::THRESH_BINARY);

        // �ϲ�
        binary = hsv | binary;

        // ��������
        // cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::GaussianBlur(binary, binary, cv::Size(3, 3), 1);
        cv::blur(binary, binary, cv::Size(3, 3));

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

            if (area > 5)
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


                if (proportion > 0.4 && proportion < 0.6)
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

            // ��ѡ����frameCenter�����װ�װ�
            drawRotatedRect(frame, closestArmor);

            // ����װ�װ�����
            cv::circle(frame, closestArmor.center, 5, cv::Scalar(255, 0, 255), -1);
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

        if (cv::waitKey(25) >= 0)
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
