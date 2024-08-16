#ifndef IO__CRANE_HPP
#define IO__CRANE_HPP

#include "io/cboard/cboard.hpp"
#include "io/command.hpp"

namespace io
{
class Crane
{
public:
  Crane(CBoard & left, CBoard & right);

  void cmd(Command command, bool left);

private:
  CBoard & left_;
  CBoard & right_;

  io::Command left_last_cmd_{0.0, 0.0, 0.0, false};
  io::Command right_last_cmd_{0.0, 0.0, 0.0, false};
};

}  // namespace io

#endif  // IO__CRANE_HPP