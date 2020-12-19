#ifndef RVIZ_UTIL_H
#define RVIZ_UTIL_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker pmarker( std::string type, 
                                    std::string mns, 
                                    unsigned id, 
                                    const std::vector<double>& scale, 
                                    const std::vector<double>& pose, 
                                    const std::vector<double>& orientation, 
                                    const std::vector<double>& color,
                                    std::string frame = "map" )
{
    visualization_msgs::Marker gen_marker;

    gen_marker.header.frame_id = frame;
    gen_marker.header.stamp = ros::Time::now();

    if( type == "cube" )
    {
        gen_marker.type = visualization_msgs::Marker::CUBE;
    }
    else
    {
        std::cout << "Error, cannot visualize marker of type: " << type << "\n";
        return gen_marker;
    }

    gen_marker.ns = mns;
    gen_marker.id = id;
    gen_marker.scale.x = scale[0];
    gen_marker.scale.y = scale[1];
    gen_marker.scale.z = scale[2];
    gen_marker.pose.position.x = pose[0];
    gen_marker.pose.position.y = pose[1];
    gen_marker.pose.position.z = pose[2];
    gen_marker.pose.orientation.x = orientation[0];
    gen_marker.pose.orientation.y = orientation[1];
    gen_marker.pose.orientation.z = orientation[2];
    gen_marker.pose.orientation.w = orientation[3];
    gen_marker.color.r = color[0];
    gen_marker.color.g = color[1];
    gen_marker.color.b = color[2];
    gen_marker.color.a = color[3];

    return gen_marker;
}

#endif
