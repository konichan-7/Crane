#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/cboard/cboard.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{x              |      | x坐标，单位m    }"
  "{y              |      | y坐标，单位m    }"
  "{z              |      | z坐标，单位m    }"
  "{grip           |      | bool          }"
  "{left l         |      | bool          }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (
    cli.has("help") || !cli.has("x") || !cli.has("y") || !cli.has("z") || !cli.has("grip") ||
    !cli.has("left")) {
    cli.printMessage();
    return 0;
  }

  auto x = cli.get<double>("x");
  auto y = cli.get<double>("y");
  auto z = cli.get<double>("z");
  auto grip = cli.get<bool>("grip");
  auto left = cli.get<bool>("left");

  io::CBoard cboard("can0", left);

  cboard.send({x, y, z, grip});
  tools::logger()->info("sent: {} {} {} {}", x, y, z, grip);

  return 0;
}