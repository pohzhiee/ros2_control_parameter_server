#include "parameter_server/parameter_server.hpp"

#include <algorithm>
#include <string>
#include <sstream>

namespace parameter_server
{
// Helper function
bool to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}


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

    auto fcn5 = std::bind(&ParameterServer::handle_GetControllerPid, this, _1, _2, _3);
    get_controller_pid_srv_ = this->create_service<GetControllerPid>("GetControllerPid", fcn5, rmw_qos_profile_services_default);
}

void ParameterServer::handle_GetAllJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<GetAllJoints::Request> request, const std::shared_ptr<GetAllJoints::Response> response)
{
    (void)request_header;
    auto param_list = this->list_parameters({""}, 10);
    unsigned int jointCount = 0;
    for (auto &paramName : param_list.names)
    {
        auto pos = paramName.find(".joints.");
        if (pos != std::string::npos)
        {
            // Get the robot name
            std::vector<size_t> dotPos = {};
            for (size_t i = 0; i < paramName.size(); i++)
            {
                if (paramName[i] == '.')
                {
                    dotPos.push_back(i);
                }
            }
            auto robotName = paramName.substr(dotPos[0] + 1, dotPos[1] - dotPos[0] - 1);
            // Check that the joints belong to the correct robot
            if (robotName.compare(request->robot) == 0)
            {
                jointCount++;
            };
        }
    }

    std::vector<std::string> jointNames = {};
    for (size_t i = 0; i < jointCount; i++)
    {
        std::string jointParamName = ".joints." + std::to_string(i);
        for (auto &paramName : param_list.names)
        {
            auto pos = paramName.find(jointParamName);
            auto length = jointParamName.size();
            auto endPos = paramName.size();
            if (pos != std::string::npos && pos == endPos - length)
            {
                // Get the robot name
                std::vector<size_t> dotPos = {};
                for (size_t i = 0; i < paramName.size(); i++)
                {
                    if (paramName[i] == '.')
                    {
                        dotPos.push_back(i);
                    }
                }
                auto robotName = paramName.substr(dotPos[0] + 1, dotPos[1] - dotPos[0] - 1);
                // Check that the joints belong to the correct robot
                if (robotName.compare(request->robot) == 0)
                {
                    // Add the joints to the list
                    auto jointParam = this->get_parameter(paramName);
                    auto jointName = jointParam.value_to_string();
                    auto unique = true;
                    for (auto &j : jointNames)
                    {
                        if (j.compare(jointName) == 0)
                            unique = false;
                    }
                    if (unique)
                        jointNames.push_back(jointName);
                }
                // skip the rest of the loop after the joint is found in the params
                continue;
            }
        }
    }
/* 
    for (auto &paramName : param_list.names)
    {
        // find parameters containing .joints.
        auto pos = paramName.find(".joints.");
        if (pos != std::string::npos)
        {
            // Get the robot name
            std::vector<size_t> dotPos = {};
            for (size_t i = 0; i < paramName.size(); i++)
            {
                if (paramName[i] == '.')
                {
                    dotPos.push_back(i);
                }
            }
            auto robotName = paramName.substr(dotPos[0] + 1, dotPos[1] - dotPos[0] - 1);
            // Check that the joints belong to the correct robot
            if (robotName.compare(request->robot) == 0)
            {
                // Add the joints to the list
                auto jointParam = this->get_parameter(paramName);
                auto jointName = jointParam.value_to_string();
                auto unique = true;
                for (auto &j : jointNames)
                {
                    if (j.compare(jointName) == 0)
                        unique = false;
                }
                if (unique)
                    jointNames.push_back(jointName);
            }
        } 
    }*/
    // for (auto &joint : jointNames)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Joint names: %s", joint.c_str());
    // }
    response->joints = jointNames;
}

void ParameterServer::handle_GetControllerJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                                 const std::shared_ptr<GetControllerJoints::Request> request, const std::shared_ptr<GetControllerJoints::Response> response)
{
    (void)request_header;
    auto param_list = this->list_parameters({""}, 10);
    std::vector<std::string> joints = {};
    for (auto &param : param_list.names)
    {
        if (param.find(request->controller + ".joints") != std::string::npos)
        {
            auto jointParam = this->get_parameter(param);
            joints.push_back(jointParam.value_to_string());
        }
    }
    response->joints = joints;
    // RCLCPP_INFO(this->get_logger(), "Joints for controller %s: ", request->controller.c_str());
    // for (auto &j : joints)
    // {
    //     RCLCPP_INFO(this->get_logger(), "%s", j.c_str());
    // }
}
void ParameterServer::handle_GetControllers(const std::shared_ptr<rmw_request_id_t> request_header,
                                            const std::shared_ptr<GetControllers::Request> request, const std::shared_ptr<GetControllers::Response> response)
{
    (void)request_header;
    auto param_list = this->list_parameters({"." + request->robot}, 10);
    std::vector<std::string> controllers = {};
    for (auto &param : param_list.names)
    {
        std::vector<size_t> dotPos = {};
        for (size_t i = 0; i < param.size(); i++)
        {
            if (param[i] == '.')
            {
                dotPos.push_back(i);
            }
        }
        auto unique = true;
        auto controllerName = param.substr(dotPos[1] + 1, dotPos[2] - dotPos[1] - 1);
        for (auto &c : controllers)
        {
            if (c.compare(controllerName) == 0)
                unique = false;
        }
        if (unique)
            controllers.push_back(controllerName);
    }
    response->controller_types = std::vector<std::string>();
    for (auto &n : controllers)
    {
        auto typeParamName = "." + request->robot + "." + n + ".type";
        auto hasTypeParam = this->has_parameter(typeParamName);
        if (hasTypeParam)
        {
            auto typeParam = this->get_parameter(typeParamName);
            response->controller_types.push_back(typeParam.value_to_string());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to obtain type for controller: %s", n.c_str());
            response->controller_types.push_back("");
        }
    }
    response->controllers = controllers;
    // for (size_t i; i < controllers.size(); i++)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Controller %d: %s [%s]", i, controllers[i], response->controller_types[i]);
    // }
}

void ParameterServer::handle_GetRobots(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<GetRobots::Request> request, const std::shared_ptr<GetRobots::Response> response)
{
    (void)request_header;
    (void)request;
    auto param_list = this->list_parameters({""}, 10);
    std::vector<std::string> robotNames = {};
    for (auto &paramName : param_list.names)
    {
        auto paramNameT = paramName.substr(1, paramName.size() - 1);
        auto iter = std::find(paramNameT.begin(), paramNameT.end(), '.');
        auto dist = std::distance(paramNameT.begin(), iter);
        auto robotName = paramNameT.substr(0, dist);
        bool unique = true;
        for (auto &str : robotNames)
        {
            if (str.compare(robotName) == 0)
                unique = false;
        }
        if (unique)
            robotNames.push_back(robotName);
    }
    // for (auto &str : robotNames)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Robot: %s", str.c_str());
    // }
    response->robots = robotNames;
}

void ParameterServer::handle_GetControllerPid(const std::shared_ptr<rmw_request_id_t> request_header,  
const std::shared_ptr<GetControllerPid::Request> request,  const std::shared_ptr<GetControllerPid::Response> response){
    (void)request_header;
    auto param_list = this->list_parameters({""}, 10);
    std::vector<std::string> joints = {};

    for (auto &param : param_list.names)
    {
        if (param.find(request->controller + ".pid") != std::string::npos)
        {
            // Get the specific parameter (p,i,d,i_min,i_max or antiwindup)
            std::vector<size_t> dotPos = {};
            for (size_t i = 0; i < param.size(); i++)
            {
                if (param[i] == '.')
                {
                    dotPos.push_back(i);
                }
            }
            // If less than 3 dots definitely not the correct parameter, so we continue to the next param
            // This also makes it such that our code below which accesses by index is safe
            if(dotPos.size() <3){
                continue;
            }
            auto endPos = dotPos.size()-1;
            auto pidParamName = param.substr(dotPos[endPos] + 1, param.size() - dotPos[endPos] - 1);
            if(pidParamName.compare("p") == 0){
                auto pidParam = this->get_parameter(param);
                auto pValStr = pidParam.value_to_string();
                RCLCPP_INFO(this->get_logger(), "p: %s", pValStr.c_str());
                response->p = std::stod(pValStr);
            }
            else if(pidParamName.compare("i") == 0){
                auto pidParam = this->get_parameter(param);
                auto iValStr = pidParam.value_to_string();
                RCLCPP_INFO(this->get_logger(), "i: %s", iValStr.c_str());
                response->i = std::stod(iValStr);
            }
            else if(pidParamName.compare("d") == 0){
                auto pidParam = this->get_parameter(param);
                auto dValStr = pidParam.value_to_string();
                RCLCPP_INFO(this->get_logger(), "d: %s", dValStr.c_str());
                response->d = std::stod(dValStr);
            }
            else if(pidParamName.compare("i_min") == 0){
                auto pidParam = this->get_parameter(param);
                auto i_minValStr = pidParam.value_to_string();
                RCLCPP_INFO(this->get_logger(), "i_min: %s", i_minValStr.c_str());
                response->i_min = std::stod(i_minValStr);
            }
            else if(pidParamName.compare("i_max") == 0){
                auto pidParam = this->get_parameter(param);
                auto i_maxValStr = pidParam.value_to_string();
                RCLCPP_INFO(this->get_logger(), "i_max: %s", i_maxValStr.c_str());
                response->i_max = std::stod(i_maxValStr);
            }
            else if(pidParamName.compare("antiwindup") == 0){
                auto pidParam = this->get_parameter(param);
                auto antiwindupValStr = pidParam.value_to_string();
                RCLCPP_INFO(this->get_logger(), "antiwindup: %s", antiwindupValStr.c_str());
                response->antiwindup = to_bool(antiwindupValStr);
            }
            else{
                auto pidParam = this->get_parameter(param);
                RCLCPP_WARN(this->get_logger(), "Unknown PID parameter: {%s: %s}", param, pidParam.value_to_string().c_str());
            }
        }
    }
}

} // namespace parameter_serverGetRobots
