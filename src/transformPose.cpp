#include <transformPose.h>

TransformPose::TransformPose(){
    planSub_ = nh_.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, &TransformPose::planCB, this);
    poseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 5, &TransformPose::poseCB, this);

    desiredPosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("/test_topic", 10);
}

void TransformPose::planCB (const moveit_msgs::DisplayTrajectory::ConstPtr& planMsg){
    plan_ = *planMsg;
}

void TransformPose::poseCB (const geometry_msgs::PoseStamped::ConstPtr& poseMsg){
    poseCurrent_ = *poseMsg;
}

void TransformPose::publishPose(geometry_msgs::PoseStamped pose){
    desiredPosePub_.publish(pose);
}

geometry_msgs::PoseStamped TransformPose::applyTransformation (){
    geometry_msgs::PoseStamped output = poseCurrent_;
    tf::Quaternion qPoseCurrent, qTrajectory, qNewRotation;
    quaternionMsgToTF(output.pose.orientation, qPoseCurrent);

    if (plan_.trajectory.size() > 0){     
        output.pose.position.x += plan_.trajectory[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x;
        output.pose.position.y += plan_.trajectory[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y;
        output.pose.position.z += plan_.trajectory[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.z;
        
        quaternionMsgToTF(plan_.trajectory[0].multi_dof_joint_trajectory.points[1].transforms[0].rotation, qTrajectory);
        qNewRotation = qPoseCurrent * qTrajectory;
        qNewRotation.normalize();
        quaternionTFToMsg(qNewRotation, output.pose.orientation);
    }

    return output;
}

TransformPose::~TransformPose(){
}

int main(int argc, char** argv){
    ros::init(argc, argv, "TransformPose");
    TransformPose transformPose;

    geometry_msgs::PoseStamped nextPose;

    ros::Rate rate(1);
    while (ros::ok()){
        ros::spinOnce();
        nextPose = transformPose.applyTransformation();
        transformPose.publishPose(nextPose);
        rate.sleep();
    }
}
