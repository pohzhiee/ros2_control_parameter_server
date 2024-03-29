#ifndef PARAMETER_SERVER__PARAMETER_SERVER_HPP_
#define PARAMETER_SERVER__PARAMETER_SERVER_HPP_

#include <memory>
#include <string>

#include "parameter_server_interfaces/srv/get_all_joints.hpp"
#include "parameter_server_interfaces/srv/get_controllers.hpp"
#include "parameter_server_interfaces/srv/get_controller_joints.hpp"
#include "parameter_server_interfaces/srv/get_controller_pid.hpp"
#include "parameter_server_interfaces/srv/get_robots.hpp"
#include "parameter_server_interfaces/srv/get_all_pid.hpp"
#include "parameter_server_interfaces/srv/get_gym_update_rate.hpp"
#include "rclcpp/rclcpp.hpp"

namespace parameter_server {
    using GetAllJoints = parameter_server_interfaces::srv::GetAllJoints;
    using GetControllerJoints = parameter_server_interfaces::srv::GetControllerJoints;
    using GetControllers = parameter_server_interfaces::srv::GetControllers;
    using GetRobots = parameter_server_interfaces::srv::GetRobots;
    using GetControllerPid = parameter_server_interfaces::srv::GetControllerPid;
    using GetAllPid = parameter_server_interfaces::srv::GetAllPid;
    using GetGymUpdateRate = parameter_server_interfaces::srv::GetGymUpdateRate;
    using namespace std::chrono_literals;

    class ParameterServer : public rclcpp::Node{
    public:
        explicit ParameterServer(
                const rclcpp::NodeOptions & options = (
                        rclcpp::NodeOptions()
                                .allow_undeclared_parameters(true)
                                .automatically_declare_parameters_from_overrides(true)));

        void load_parameters(const std::string & yaml_config_file);

        void load_parameters(const std::string & key, const std::string & value);
    private:
        rclcpp::Service<GetAllJoints>::SharedPtr get_all_joints_srv_;
        rclcpp::Service<GetControllerJoints>::SharedPtr get_controller_joints_srv_;
        rclcpp::Service<GetControllers>::SharedPtr get_controllers_srv_;
        rclcpp::Service<GetRobots>::SharedPtr get_robots_srv_;
        rclcpp::Service<GetControllerPid>::SharedPtr get_controller_pid_srv_;
        rclcpp::Service<GetAllPid>::SharedPtr get_all_pid_srv_;
        rclcpp::Service<GetGymUpdateRate>::SharedPtr get_gym_update_rate_srv;

        void handle_GetAllJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<GetAllJoints::Request> request,
                                 const std::shared_ptr<GetAllJoints::Response> response);

        void handle_GetControllerJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                        const std::shared_ptr<GetControllerJoints::Request> request,
                                        const std::shared_ptr<GetControllerJoints::Response> response);

        void handle_GetControllers(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<GetControllers::Request> request,
                                   const std::shared_ptr<GetControllers::Response> response);

        void handle_GetRobots(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<GetRobots::Request> request,
                              const std::shared_ptr<GetRobots::Response> response);

        void handle_GetControllerPid(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<GetControllerPid::Request> request,
                                     const std::shared_ptr<GetControllerPid::Response> response);

        void handle_GetAllPid(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<GetAllPid::Request> request,
                              const std::shared_ptr<GetAllPid::Response> response);

        void handle_GetGymUpdateRate(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<GetGymUpdateRate::Request> request,
                                     const std::shared_ptr<GetGymUpdateRate::Response> response);



    };

}  // namespace parameter_server

#endif  // PARAMETER_SERVER__PARAMETER_SERVER_HPP_
