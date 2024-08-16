#include "crane.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace io
{
Crane::Crane(CBoard & left, CBoard & right) : left_(left), right_(right) {}

void Crane::cmd(Command command, bool left)
{
  if (left) {
    left_.send(command);
    left_last_cmd_ = command;
    return;
  }

  auto left_cmd = left_last_cmd_;
  left_cmd.x = command.x;
  left_.send(left_cmd);
  left_last_cmd_ = left_cmd;

  right_.send(command);
  right_last_cmd_ = command;
}

}  // namespace io
