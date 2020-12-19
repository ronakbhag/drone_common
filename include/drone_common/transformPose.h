#ifndef TRANSFORM_POSE
#define TRANSFORM_POSE

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>

class TransformPose {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber planSub_;
        ros::Subscriber poseSub_;
        ros::Publisher desiredPosePub_;

        moveit_msgs::DisplayTrajectory plan_;
        geometry_msgs::PoseStamped poseCurrent_;

    public:
        TransformPose();
        ~TransformPose();
        
        void planCB(const moveit_msgs::DisplayTrajectory::ConstPtr& planMsg);
        void poseCB(const geometry_msgs::PoseStamped::ConstPtr& poseMsg);
        void publishPose(geometry_msgs::PoseStamped);

        geometry_msgs::PoseStamped applyTransformation();

};

#endif
