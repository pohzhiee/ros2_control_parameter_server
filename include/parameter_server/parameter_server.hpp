#ifndef PARAMETER_SERVER__PARAMETER_SERVER_HPP_
#define PARAMETER_SERVER__PARAMETER_SERVER_HPP_

#include <memory>
#include <string>

#include "controller_parameter_server/parameter_server.hpp"
#include "parameter_server_interfaces/srv/get_all_joints.hpp"
#include "parameter_server_interfaces/srv/get_controllers.hpp"
#include "parameter_server_interfaces/srv/get_controller_joints.hpp"
#include "parameter_server_interfaces/srv/get_robots.hpp"
#include "rclcpp/rclcpp.hpp"

namespace parameter_server
{
  using GetAllJoints = parameter_server_interfaces::srv::GetAllJoints;
  using GetControllerJoints = parameter_server_interfaces::srv::GetControllerJoints;
  using GetControllers = parameter_server_interfaces::srv::GetControllers;
  using GetRobots = parameter_server_interfaces::srv::GetRobots;
  using namespace std::chrono_literals;

class ParameterServer : public controller_parameter_server::ParameterServer
{
public:
  ParameterServer();

  virtual
  ~ParameterServer() = default;
private:
  rclcpp::Service<GetAllJoints>::SharedPtr get_all_joints_srv_;
  rclcpp::Service<GetControllerJoints>::SharedPtr get_controller_joints_srv_;
  rclcpp::Service<GetControllers>::SharedPtr get_controllers_srv_;
  rclcpp::Service<GetRobots>::SharedPtr get_robots_srv_;


void handle_GetAllJoints(const std::shared_ptr<rmw_request_id_t> request_header,  
const std::shared_ptr<GetAllJoints::Request> request,  const std::shared_ptr<GetAllJoints::Response> response);

void handle_GetControllerJoints(const std::shared_ptr<rmw_request_id_t> request_header,  
const std::shared_ptr<GetControllerJoints::Request> request,  const std::shared_ptr<GetControllerJoints::Response> response);

void handle_GetControllers(const std::shared_ptr<rmw_request_id_t> request_header,  
const std::shared_ptr<GetControllers::Request> request,  const std::shared_ptr<GetControllers::Response> response);

void handle_GetRobots(const std::shared_ptr<rmw_request_id_t> request_header,  
const std::shared_ptr<GetRobots::Request> request,  const std::shared_ptr<GetRobots::Response> response);

};

}  // namespace parameter_server

#endif  // PARAMETER_SERVER__PARAMETER_SERVER_HPP_
