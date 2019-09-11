#include "parameter_server/parameter_server.hpp"

#include <algorithm>
#include <string>

namespace parameter_server
{
using namespace std::placeholders;
ParameterServer::ParameterServer() : controller_parameter_server::ParameterServer()
{
    auto fcn = std::bind(&ParameterServer::handle_GetAllJoints, this, _1, _2, _3);
    get_all_joints_srv_ = this->create_service<GetAllJoints>("GetAllControlJoints", fcn, rmw_qos_profile_services_default);

    auto fcn2 = std::bind(&ParameterServer::handle_GetControllerJoints, this, _1, _2, _3);
    get_controller_joints_srv_ = this->create_service<GetControllerJoints>("GetControllerJoints", fcn2, rmw_qos_profile_services_default);

    auto fcn3 = std::bind(&ParameterServer::handle_GetControllers, this, _1, _2, _3);
    get_controllers_srv_ = this->create_service<GetControllers>("GetControllers", fcn3, rmw_qos_profile_services_default);

    auto fcn4 = std::bind(&ParameterServer::handle_GetRobots, this, _1, _2, _3);
    get_robots_srv_ = this->create_service<GetRobots>("GetRobots", fcn4, rmw_qos_profile_services_default);
}

void ParameterServer::handle_GetAllJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<GetAllJoints::Request> request, const std::shared_ptr<GetAllJoints::Response> response)
{
    (void) request_header;
    (void) request;
    auto param_list = this->list_parameters({""}, 10);

    std::vector<std::string> jointNames = {};
    for(auto &paramName : param_list.names){
        auto pos =  paramName.find(".joints.");
        if(pos != std::string::npos){
            auto jointParam = this->get_parameter(paramName);
            auto jointName = jointParam.value_to_string();
            auto unique = true;
            for(auto &j : jointNames){
                if(j.compare(jointName) == 0) unique = false;
            }
            if(unique) jointNames.push_back(jointName);
        }
    }
    for(auto &joint : jointNames){
        RCLCPP_INFO(this->get_logger(), "Joint names: %s", joint.c_str());
    }
    response ->joints = jointNames;
}

void ParameterServer::handle_GetControllerJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                                 const std::shared_ptr<GetControllerJoints::Request> request, const std::shared_ptr<GetControllerJoints::Response> response)
{
    (void) request_header;
    (void) response;
    auto param_list = this->list_parameters({""}, 10);
    std::vector<std::string> joints = {};
    for (auto &param : param_list.names)
    {
        if(param.find(request->controller_name+".joints") != std::string::npos){
            auto jointParam = this->get_parameter(param);
            joints.push_back(jointParam.value_to_string());
        }
    }
    response->joints = joints;
    RCLCPP_INFO(this->get_logger(), "Joints for controller %s: ", request->controller_name.c_str());
    for(auto &j : joints){
        RCLCPP_INFO(this->get_logger(), "%s", j.c_str());
    }
}

void ParameterServer::handle_GetControllers(const std::shared_ptr<rmw_request_id_t> request_header,
                                            const std::shared_ptr<GetControllers::Request> request, const std::shared_ptr<GetControllers::Response> response)
{
    (void) request_header;
    auto param_list = this->list_parameters({"."+request->robot}, 10);
    std::vector<std::string> controllers = {};
    for (auto &param : param_list.names)
    {
        std::vector<size_t> dotPos = {};
        for(size_t i = 0;i<param.size();i++){
            if(param[i]=='.'){
                dotPos.push_back(i);
            }
        }
        auto unique = true;
        auto controllerName = param.substr(dotPos[1]+1, dotPos[2]-dotPos[1]-1);
        for(auto &c : controllers){
            if(c.compare(controllerName) == 0)  unique = false;
        }
        if(unique) controllers.push_back(controllerName);
    }
    response->controller_types = std::vector<std::string>();
    for(auto &n : controllers){
        auto typeParamName = "." + request->robot + "." + n + ".type";
        auto hasTypeParam = this->has_parameter(typeParamName);
        if(hasTypeParam) {
            auto typeParam = this->get_parameter(typeParamName);
            response->controller_types.push_back(typeParam.value_to_string());
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Unable to obtain type for controller: %s", n.c_str());
            response->controller_types.push_back("");
        }
    }
    response->controllers = controllers;
    for(size_t i;i<controllers.size();i++){
        RCLCPP_INFO(this->get_logger(),"Controller %d: %s [%s]", i, controllers[i], response->controller_types[i]);
    }
}

void ParameterServer::handle_GetRobots(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<GetRobots::Request> request, const std::shared_ptr<GetRobots::Response> response)
{
    (void) request_header;
    (void) request;
    auto param_list = this->list_parameters({""},10);
    std::vector<std::string> robotNames = {};
    for (auto &paramName : param_list.names)
    {
        auto paramNameT = paramName.substr(1, paramName.size()-1);
        auto iter = std::find(paramNameT.begin(), paramNameT.end(), '.');
        auto dist = std::distance(paramNameT.begin(), iter);
        auto robotName = paramNameT.substr(0, dist);
        bool unique = true;
        for (auto &str : robotNames)
        {
            if (str.compare(robotName) == 0) unique = false;
        }
        if (unique) robotNames.push_back(robotName);
    }
    for(auto &str : robotNames){
        RCLCPP_INFO(this->get_logger(), "Robot: %s", str.c_str());
    }
    response->robots = robotNames;
}

} // namespace parameter_server