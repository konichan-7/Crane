#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  double x;
  double y;
  double z;
  bool grip;
  bool slow;
};

}  // namespace io

#endif  // IO__COMMAND_HPP