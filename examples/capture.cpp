#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "io/usbcamera/usbcamera.hpp"

const std::string keys =
    "{help h usage ?  |                     | 输出命令行参数说明}"
    "{name n          |video0| 端口名称 }"
    "{config-path c   | configs/usbcamera.yaml | yaml配置文件路径 }"
    "{output-folder o | assets/img_with_q   | 输出文件夹路径   }";

void capture_loop(
    const std::string &config_path, const std::string &device_name, const std::string &output_folder)
{
    io::USBCamera usbcam(device_name, config_path);
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    int count = 0;
    while (true)
    {

        img = usbcam.read();

        // 按“s”保存图片和对应四元数，按“q”退出程序
        cv::imshow("Press s to save, q to quit", img);
        auto key = cv::waitKey(1);
        if (key == 'q')
            break;
        else if (key != 's')
            continue;

        // 保存图片和四元数
        count++;
        auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
        cv::imwrite(img_path, img);
        tools::logger()->info("[{}] Saved in {}", count, output_folder);
    }

    // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char *argv[])
{
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help"))
    {
        cli.printMessage();
        return 0;
    }

    auto output_folder = cli.get<std::string>("output-folder");
    auto device_name = cli.get<std::string>("name");
    auto config_path = cli.get<std::string>("config-path");

    // 新建输出文件夹
    std::filesystem::create_directory(output_folder);

    // 主循环，保存图片和对应四元数
    capture_loop(config_path, device_name, output_folder);

    tools::logger()->info("finish capture");

    return 0;
}
