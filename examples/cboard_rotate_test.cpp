#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{dx             |      | 单位m    }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("dx")) {
    cli.printMessage();
    return 0;
  }

  auto dx = cli.get<double>("dx");

  io::CBoard cboard("can0", true);

  cboard.rotate(dx);
  tools::logger()->info("rotate {}", dx);

  return 0;
}