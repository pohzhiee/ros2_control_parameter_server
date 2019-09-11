#ifndef PARAMETER_SERVER__PARAMETER_SERVER_HPP_
#define PARAMETER_SERVER__PARAMETER_SERVER_HPP_

#include <memory>
#include <string>

#include "controller_parameter_server/parameter_server.hpp"

#include "rclcpp/rclcpp.hpp"

namespace parameter_server
{

class ParameterServer : public controller_parameter_server::ParameterServer
{
public:
  ParameterServer();

  virtual
  ~ParameterServer() = default;

};

}  // namespace parameter_server

#endif  // PARAMETER_SERVER__PARAMETER_SERVER_HPP_
