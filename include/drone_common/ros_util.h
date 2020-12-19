#ifndef ROS_UTIL_H
#define ROS_UTIL_H

#include <ros/ros.h>

template <typename T>
inline T GetParam(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
{
    T param_val;
    if( !nh.getParam(param_name, param_val) )
    {
        param_val = default_val;
        ROS_WARN_STREAM_NAMED("params", "Defaulting " << param_name << " to " << param_val);
    }
    return param_val;
}

template <typename T>
inline T GetParam(const ros::NodeHandle& nh, const std::string& param_name, T&& default_val)
{
    T param_val;
    if( !nh.getParam(param_name, param_val) )
    {
        param_val = default_val;
        ROS_WARN_STREAM_NAMED("params", "Defaulting " << param_name << " to " << param_val);
    }
    return param_val;
}

template <typename T>
inline T GetParam(const ros::NodeHandle& nh, const std::string& param_name)
{
    T param_val;
    if( !nh.getParam(param_name, param_val) )
    {
        std::string er = std::string("[") + ros::this_node::getName() + "/" + nh.getNamespace() + std::string("] Could not find parameter [") + param_name + std::string("]");
        throw std::runtime_error( er );
    }
    return param_val;
}


#endif
