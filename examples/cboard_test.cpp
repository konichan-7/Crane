#include "io/cboard/cboard.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{left           |      | bool          }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("left")) {
    cli.printMessage();
    return 0;
  }

  auto left = cli.get<bool>("left");

  tools::Exiter exiter;

  io::CBoard cboard("can0", left);

  while (!exiter.exit()) {
    auto t = std::chrono::steady_clock::now();

    Eigen::Vector3d xyz = cboard.odom_at(t);
    tools::logger()->info("{:.3f} {:.3f} {:.3f}", xyz[0], xyz[1], xyz[2]);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}