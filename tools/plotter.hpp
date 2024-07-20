#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <string>
#include <nlohmann/json.hpp>

namespace tools
{
class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  ~Plotter();

  void plot(const nlohmann::json & json);

private:
  int socket_;
  sockaddr_in destination_;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP