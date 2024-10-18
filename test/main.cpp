#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath> 

// 定义Lightbar灯条结构体
struct Lightbar
{
    cv::Point2f center;  // 灯条中心点
    float width;         // 灯条宽度
    float height;        // 灯条高度
};

// 定义Armor装甲板结构体
struct Armor
{
    cv::Point2f center;       // 装甲板中心点
    Lightbar lightbar1;       // 灯条1
    Lightbar lightbar2;       // 灯条2
};

// 根据armor框选装甲板
void drawRotatedRect(cv::Mat& image, const Armor& armor)
{
    // 计算灯条之间的角度
    float angle = std::atan2(armor.lightbar2.center.y - armor.lightbar1.center.y, armor.lightbar2.center.x - armor.lightbar1.center.x) * 180.0 / CV_PI;
   
    float width = cv::norm(armor.lightbar2.center - armor.lightbar1.center);
    float height = ((armor.lightbar1.height + armor.lightbar2.height) / 2) * 2.2;

    // 构造旋转矩形
    cv::RotatedRect rotatedRect(armor.center, cv::Size2f(width, height), angle);

    // 获取矩形的四个顶点
    cv::Point2f vertices[4];
    rotatedRect.points(vertices);

    // 绘制矩形
    for (int i = 0; i < 4; ++i)
        cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
}

int main()
{
    // 白色HSV
    cv::Scalar lowerWhite(0, 0, 200);
    cv::Scalar upperWhite(180, 60, 255);

    // 打开视频文件
    cv::VideoCapture cap("1.mp4"); 
    if (!cap.isOpened())
    {
        std::cerr << "读取失败" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true)
    {
        // 从视频中读取每一帧
        cap >> frame;
        if (frame.empty())
            break;

        // 获取帧的宽度
        int width = frame.cols;
        // 获取帧的高度
        int height = frame.rows;

        cv::Point2f frameCenter(width / 2.0f, height / 2.0f);

        // 提取灯条
        cv::Mat hsv, binary, gray;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lowerWhite, upperWhite, hsv);

        // 转换为灰度图
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 提取灯条白色部分
        cv::threshold(gray, binary, 230, 255, cv::THRESH_BINARY);

        // 合并
        binary = hsv | binary;

        // 构造卷积核
        // cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        cv::GaussianBlur(binary, binary, cv::Size(3, 3), 1);
        cv::blur(binary, binary, cv::Size(3, 3));

        // 获取轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 定义灯条结构体列表
        std::vector<Lightbar> lightbars;

        // 筛选灯条
        for (const auto& contour : contours)
        {
            float area = cv::contourArea(contour);
            cv::Rect boundingBox = cv::boundingRect(contour);

            if (area > 5)
            {
                // 创建灯条对象
                Lightbar lightbar;
                lightbar.center = (boundingBox.tl() + boundingBox.br()) * 0.5;
                lightbar.width = static_cast<float>(boundingBox.width);
                lightbar.height = static_cast<float>(boundingBox.height);
                // 存储灯条
                lightbars.push_back(lightbar);
            }
        }

        // 定义灯条配对状态
        std::vector<bool> paired(lightbars.size(), false);

        // 定义装甲板
        std::vector<Armor> armors;

        // 进行灯条配对，生成装甲板
        for (size_t i = 0; i < lightbars.size(); i++)
        {
            if (paired[i]) continue;

            for (size_t j = i + 1; j < lightbars.size(); j++)
            {
                if (paired[j]) continue;

                // 计算灯条距离
                float lightDistance = cv::norm(lightbars[i].center - lightbars[j].center);
                
                //测试灯条长度和装甲板长度比例范围
                //std::cout << lightbars[i].height / lightDistance << std::endl;
                float proportion = lightbars[i].height / lightDistance; 


                if (proportion > 0.4 && proportion < 0.6)
                {
                    // 创建一个装甲板对象
                    Armor armor;
                    armor.center = (lightbars[i].center + lightbars[j].center) * 0.5;
                    armor.lightbar1 = lightbars[i];
                    armor.lightbar2 = lightbars[j];

                    // 保存装甲板数据
                    armors.push_back(armor);

                    paired[i] = true;
                    paired[j] = true;

                    break;
                }
            }
        }

        // 找出距离中心最近的装甲板
        if (!armors.empty())
        {
            // 先假设第一个装甲板为最接近中心的
            Armor closestArmor = armors[0];
            // 定义最小距离,初始值为第一个装甲板的数据
            float minDistance = cv::norm(closestArmor.center - frameCenter);

            // 遍历所有存在的装甲板
            for (const auto& armor : armors)
            {
                // 通过坐标计算距离
                float distance = cv::norm(armor.center - frameCenter);
                if (distance < minDistance)
                {
                    closestArmor = armor;
                    minDistance = distance;
                }
            }

            // 框选距离frameCenter最近的装甲板
            drawRotatedRect(frame, closestArmor);

            // 绘制装甲板中心
            cv::circle(frame, closestArmor.center, 5, cv::Scalar(255, 0, 255), -1);
        }

        /*
        // 测试(输出paired)
        for (size_t i = 0; i < paired.size(); ++i)
        {
            std::cout << "paired[" << i << "] = " << paired[i] << std::endl;
        }
        */

        /*
        // 测试(输出armors)
        for (size_t i = 0; i < armors.size(); ++i)
        {
            std::cout << "armer[" << i << "].center = " << armors[i].center << std::endl;
            std::cout << "armer[" << i << "].lightbar1 = " << armors[i].lightbar1.center << std::endl;
            std::cout << "armer[" << i << "].lightbar2 = " << armors[i].lightbar2.center << std::endl;
        }
        */

        /*
        // 测试(输出lightbars)
        for (size_t i = 0; i < lightbars.size(); ++i)
        {
            std::cout << "lightbars[" << i << "] = " << lightbars[i].center << std::endl;
        }
        */

        // 显示结果图像
        cv::imshow("main", frame);

        // 显示测试图像
        // cv::imshow("binary", binary);

        if (cv::waitKey(25) >= 0)
            break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
