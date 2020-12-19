#ifndef DRONE_H
#define DRONE_H

/* Class created to encapsulate Drone commands such as
 * arming and setting the mode of the drone. The class
 * keeps track of the current state of the drone and
 * has two service clients for arming and setting the
 * mode.
 */

#include <ros/ros.h>
#include "ros_util.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>

#include <string>

namespace drone_interface
{
    class Drone
    {
        private:
            ros::NodeHandle nh_;

            // Subscribers for the state, pose, odom of the drone
            ros::Subscriber stateSub_;
            ros::Subscriber poseSub_;
            ros::Subscriber odomSub_;
            ros::Subscriber visionSub_;

            // Publisher for plugin to be used with MAVROS
            ros::Publisher localPub_;

            // Service clients for arming and setting mode
            ros::ServiceClient armingClient_;
            ros::ServiceClient setModeClient_;

            // Members to hold state and pose information
            mavros_msgs::State droneCurrentState_;
            nav_msgs::Odometry droneCurrentOdom_;
            geometry_msgs::PoseStamped droneCurrentVision_;
            geometry_msgs::PoseStamped droneCurrentPose_;
            geometry_msgs::Pose accessPose_;

            // Mutices for preventing race conditions
            std::mutex position_mx;
            std::mutex state_mx;

            // Initialization Flags
            bool use_vision = false;
            bool use_odom = false;
            bool self_arm = false;
            bool autonomous_mode = false;

            // Time Data
            ros::Time last_vision_time;
            ros::Time last_odom_time;

        public:
            Drone();
            ~Drone();

            // Function for checking drone readiness given init flags
            bool ready();

            // Initialization functions
            void setInitFlags( std::vector<std::string> flags );
            bool Initialize();

            // Getting the last known pose of the drone
            geometry_msgs::Pose getPose();

            // Callbacks for reading information over MAVROS
            void stateCB(const mavros_msgs::State::ConstPtr& stateMsg);
            void poseCB(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
            void visionCB(const geometry_msgs::PoseStamped::ConstPtr& visionMsg);

            // Functions encapsulating arming and mode changing
            bool setMode(const std::string mode);
            bool arm();

            // Calculate distance to goal using distance formula in three dimensions
            double distanceToPoint(const geometry_msgs::PoseStamped pose);

            // Getters
            mavros_msgs::State getCurrentState();
            void reportCurrentState();
    };
}

#endif
