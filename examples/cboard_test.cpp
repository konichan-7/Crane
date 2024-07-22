#include "io/cboard.hpp"

#include <chrono>
#include <thread>

#include "io/command.hpp"
#include "tools/exiter.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;

  io::CBoard cboard("can0");
  io::Command command{0, 0, 0};

  while (!exiter.exit()) {
    std::this_thread::sleep_for(5ms);

    cboard.send(command);
  }

  return 0;
}