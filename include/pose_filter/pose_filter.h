#ifndef pose_filter_H
#define pose_filter_H

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

/*
ndt_pose...PoseStamped
amcl_pose...PoseWithCovarianceStamped
*/
   
class PoseFilter{
public:
    PoseFilter();
    ~PoseFilter();
    void msgsCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs);
    void timerCallback(const ros::TimerEvent&);
   
private:
    ros::Publisher pose_pub_;
    ros::Subscriber pose_sub_;
    ros::Timer timer_;

    geometry_msgs::PoseWithCovarianceStamped pose_msg_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Pose new_pose_;

    void broadcastFrame();
    void publishPose();

    std::string pub_topic_;
    std::string sub_topic_;
    std::string global_frame_;
    std::string base_frame_;

};

#endif