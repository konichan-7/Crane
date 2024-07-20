#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "io/usbcamera/usbcamera.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono;
using namespace std;

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{name n         | video0| 端口名称 }"
    "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char *argv[])
{
    tools::Exiter exiter;
    tools::Plotter plotter;

    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help"))
    {
        cli.printMessage();
        return 0;
    }

    auto device_name = cli.get<std::string>("name");
    auto config_path = cli.get<std::string>(0);

    io::USBCamera usbcam(device_name, config_path);

    cv::Mat img;
    cv::Mat gray_img;
    cv::Mat binary_img;

    for (int frame_count = 0; !exiter.exit(); frame_count++)
    {

        img = usbcam.read();
        cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        // cv::threshold(gray_img, binary_img, 150, 255, cv::THRESH_BINARY);

        cv::adaptiveThreshold(gray_img, binary_img, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 21, -2.5);
        cv::blur(binary_img, binary_img, cv::Size(3, 3));
        // cv::morphologyEx(binary_img, binary_img, cv::MORPH_CLOSE, element);

        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        vector<cv::RotatedRect> rrts;

        for (size_t i = 0; i < contours.size(); i++)
        {
            // 检查轮廓点数量
            if (contours[i].size() < 5)
                continue;

            // 拟合椭圆
            cv::RotatedRect rrt = fitEllipse(contours[i]);

            // 检查椭圆尺寸
            if (rrt.size.width > 0 && rrt.size.height > 0 && rrt.size.area() > 10000 && rrt.size.area() < 100000)
            {
                float axis_ratio = max(rrt.size.width, rrt.size.height) / min(rrt.size.width, rrt.size.height);

                // 根据长短轴长度之比进行筛选
                if (axis_ratio < 1.8)
                {
                    cv::ellipse(img, rrt, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    cv::Point2f center = rrt.center;
                    cv::circle(img, center, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
                    rrts.push_back(rrt);
                }
            }
        }
        // for(auto i:rrts){

        // }
        cv::resize(binary_img, binary_img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
        // cv::resize(img, img, {}, 0.5, 0.5);               // 显示时缩小图片尺寸
        cv::resize(gray_img, gray_img, {}, 0.5, 0.5);
        cv::imshow("binary", binary_img);
        cv::imshow("bgrimg", img);
        cv::imshow("gray", gray_img);
        auto key = cv::waitKey(33);
        if (key == 'q')
            break;
    }

    return 0;
}
