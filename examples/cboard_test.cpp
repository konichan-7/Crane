#include "io/cboard/cboard.hpp"

#include <chrono>
#include <thread>

#include "tools/exiter.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;

  io::CBoard cboard("can0");

  while (!exiter.exit()) {
    auto t = std::chrono::steady_clock::now();

    Eigen::Vector3d xyz = cboard.odom_at(t);
    bool servo_state = cboard.servo();
    tools::logger()->info("{:.3f} {:.3f} {:.3f} {}", xyz[0], xyz[1], xyz[2], servo_state);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}