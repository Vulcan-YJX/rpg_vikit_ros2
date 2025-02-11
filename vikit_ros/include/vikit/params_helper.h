// Copyright (c) 2023 VulcanYJX
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

inline 
bool hasParam(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
    return node->has_parameter(name);
}

template<typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& defaultValue)
{
    T value;
    if (node->get_parameter(name, value))  // 尝试获取参数
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << value);
        return value;
    }
    else
    {
        RCLCPP_WARN_STREAM(node->get_logger(), "Cannot find value for parameter: " << name 
                                               << ", assigning default: " << defaultValue);
        return defaultValue;
    }
}

template<typename T>
T getParam(const std::string& name)
{
  T v;
  int i = 0;
  while(ros::param::get(name, v) == false)
  {
    ROS_ERROR_STREAM("Cannot find value for parameter: " << name << ", will try again.");
    if ((i ++) >= 5) return T();
  }
  
  ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
  return v;
}

template<typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
    T value;
    int retries = 0;
    const int max_retries = 5;  // 最大重试次数
    while (!node->get_parameter(name, value))  // 尝试获取参数
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find value for parameter: " << name 
                                                 << ", will try again.");
        if (++retries >= max_retries)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Max retries reached for parameter: " << name 
                                                     << ". Returning default-constructed value.");
            return T();  // 返回默认构造的值
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待 1 秒后重试
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << value);
    return value;
}


} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
