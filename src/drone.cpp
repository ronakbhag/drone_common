#include "drone.h"

namespace drone_interface
{
    /**
     *
     */
    Drone::Drone()
    {
        // Initialize all the subscribers, publisher, and service clients
        stateSub_ = nh_.subscribe<mavros_msgs::State>        ("/mavros/state", 20, &Drone::stateCB, this);
        poseSub_ = nh_.subscribe<geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 20, &Drone::poseCB, this);

        armingClient_ = nh_.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
        setModeClient_ = nh_.serviceClient<mavros_msgs::SetMode>    ("/mavros/set_mode");
    }

    /**
     *
     */
    Drone::~Drone(){};

    /**
     *
     */
    bool Drone::ready()
    {
        // Check for updated times
        if( use_vision && ( ros::Time::now() - last_vision_time ).toSec() > 0.25 )
            return false;
        if( use_odom && ( ros::Time::now() - last_odom_time ).toSec() > 0.25 )
            return false;
        // Check that the drone is armed
        if( !getCurrentState().armed )
            return false;
        // And make sure we are in offboard to send commands

        // Everything checks out, say we're ready
        return true;
    }

    /**
     *
     */
    void Drone::setInitFlags( std::vector<std::string> flags )
    {
        //We're just going to search for flags
        /* Vision */
        if( std::find (flags.begin(), flags.end(), "vision") != flags.end() )
            use_vision = true;
        /* Odom */
        if( std::find (flags.begin(), flags.end(), "odom") != flags.end() )
            use_odom = true;
        /* Auto-arming */
        if( std::find (flags.begin(), flags.end(), "arm") != flags.end() )
            self_arm = true;
        /* Autonomy */
        if( std::find (flags.begin(), flags.end(), "autonomous") != flags.end() )
            autonomous_mode = true;
    }

    /**
     *
     */
    bool Drone::Initialize()
    {
        //Wait for the drone to see the world
        ros::Rate rate(5.0);

        // We can't use both vision and odom, so make sure we're not doing that.
        if( use_vision && use_odom )
        {
            ROS_ERROR("[DRONE] Cannot use both Vision and Odometry!");
            return false;
        }

        // We need to get some sort of external information
        bool position_acquired = false;

        /* VISION */
        if( use_vision )
        {
            ROS_INFO("[DRONE] Path Follower Waiting for Vision data...");
            // We also need to subscribe to the vision
            visionSub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 20, &Drone::visionCB, this);
            // Then wait until we have a response
            while( ros::ok() && !position_acquired )
            {
                ros::spinOnce();
                position_acquired = (ros::Time::now() - last_vision_time).toSec() < 0.25;
                rate.sleep();
            }
            ROS_INFO("[DRONE] Vision is live, ready to arm the drone...");
        }

        /* ODOMETRY */
        if( use_odom )
        {
            ROS_INFO("[DRONE] Path Follower Waiting for Odometry data...");
            // Also subscribe to the odometry
            odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 20, &Drone::odomCB, this);
            // And wait for a position to come across
            while( ros::ok() && !position_acquired )
            {
                ros::spinOnce();
                position_acquired = (ros::Time::now() - last_odom_time).toSec() < 0.25;
                rate.sleep();
            }
            ROS_INFO("[DRONE] Odometry is live, ready to arm the drone...");
        }

        //Attempt to arm the drone if we're self-starting, otherwise wait for arming
        while( ros::ok() && !getCurrentState().armed )
        {
            ros::spinOnce();
            if( self_arm )
            {
                arm();
            }
            rate.sleep();
        }
        ROS_INFO("[DRONE] Drone Armed and we have Position Lock!");

        //Finally, before we begin sending new commands, we need to wait for
        //   the drone to be in offboard
        while( ros::ok() && getCurrentState().mode != "OFFBOARD" )
        {
            ros::spinOnce();
            if(autonomous_mode)
            {
                setMode("OFFBOARD");
            }
            rate.sleep();
        }

        // We don't want to say initialization worked if ROS is not okay.
        if( ros::ok() )
            return true;
        return false;
    }

    /**
     *
     */
    geometry_msgs::Pose Drone::getPose()
    {
        std::lock_guard<std::mutex> lock(position_mx);
        return accessPose_;
    }

    /**
     *
     */
    void Drone::stateCB (const mavros_msgs::State::ConstPtr& stateMsg)
    {
        std::lock_guard<std::mutex> lock(state_mx);
        droneCurrentState_ = *stateMsg;
    }

    /**
     *
     */
    void Drone::poseCB (const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
    {
        droneCurrentPose_ = *poseMsg;
        std::lock_guard<std::mutex> lock(position_mx);
        accessPose_ = poseMsg->pose;
    }

    /**
     *
     */
    void Drone::odomCB (const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        droneCurrentOdom_ = *odomMsg;
        last_odom_time = odomMsg->header.stamp;
        // Lock and save pose
        std::lock_guard<std::mutex> lock(position_mx);
        accessPose_ = odomMsg->pose.pose;
    }

    /**
     *
     */
    void Drone::visionCB (const geometry_msgs::PoseStamped::ConstPtr& visionMsg)
    {
        droneCurrentVision_ = *visionMsg;
        last_vision_time = visionMsg->header.stamp;
        // Lock and save pose
        std::lock_guard<std::mutex> lock(position_mx);
        accessPose_ = visionMsg->pose;
    }

    /**
     *
     */
    bool Drone::setMode(const std::string mode)
    {
        mavros_msgs::SetMode setModeMsg;
        setModeMsg.request.custom_mode = mode;
        setModeClient_.call(setModeMsg);

        return setModeMsg.response.mode_sent;
    }

    /**
     *
     */
    bool Drone::arm()
    {
        mavros_msgs::CommandBool armCmd;
        armCmd.request.value = true;
        armingClient_.call(armCmd);

        return armCmd.response.success;
    }

    /**
     *
     */
    mavros_msgs::State Drone::getCurrentState()
    {
        std::lock_guard<std::mutex> lock(state_mx);
        return droneCurrentState_;
    }

    /**
     *
     */
    void Drone::reportCurrentState()
    {
        std::lock_guard<std::mutex> lock(state_mx);
        ROS_WARN_STREAM("Reporting Drone State [" << droneCurrentState_.header.stamp << "]");
        ROS_INFO_STREAM("Mode [" << droneCurrentState_.mode << "]");
        ROS_INFO_STREAM("Conn [" << (droneCurrentState_.connected ? "True" : "False") << "]");
        ROS_INFO_STREAM("Armd [" << (droneCurrentState_.armed ? "True" : "False") << "]");
        ROS_INFO_STREAM("RCCC [" << (droneCurrentState_.guided ? "True" : "False") << "]");
    }

}
