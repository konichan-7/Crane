#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
  struct Command
  {
    int state;
    double x_bias;
    double y_bias;
  };

} // namespace io

#endif // IO__COMMAND_HPP