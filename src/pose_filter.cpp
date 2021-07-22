#include "pose_filter/pose_filter.h"
   
PoseFilter::PoseFilter(){
    ros::NodeHandle nh("~");
    nh.param("global_frame",    global_frame_,  std::string("map"));
    nh.param("base_frame",      base_frame_,    std::string("base_link"));
    nh.param("pub_topic",       pub_topic_,     std::string("amcl_pose"));
    nh.param("sub_topic",       sub_topic_,     std::string("ndt_pose"));
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_topic_, 10);
    pose_sub_ = nh.subscribe(sub_topic_, 1, &PoseFilter::msgsCallback, this);
    timer_ = nh.createTimer(ros::Duration(0.05), &PoseFilter::timerCallback, this);

    pose_.position.x        = 0.0;
    pose_.position.y        = 0.0;
    pose_.position.z        = 0.0;
    new_pose_.position.x    = 0.0;
    new_pose_.position.y    = 0.0;
    new_pose_.position.z    = 0.0;
    new_pose_.orientation.x = 0.0;
    new_pose_.orientation.y = 0.0;
    new_pose_.orientation.z = 0.0;
    new_pose_.orientation.w = 1.0;
    
}

PoseFilter::~PoseFilter(){

}

void PoseFilter::msgsCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs){
    pose_ = msgs->pose;
    new_pose_.position.x = pose_.position.x;
    new_pose_.position.y = pose_.position.y;
    new_pose_.position.z = pose_.position.z;
    new_pose_.orientation = pose_.orientation;

    publishPose();
}

void PoseFilter::timerCallback(const ros::TimerEvent&){
    broadcastFrame();
    // ROS_INFO("x:%0.3f, y:%0.3f", pose_.position.x, pose_.position.y);
}

void PoseFilter::publishPose(){
    static unsigned int seq = 0;
    ++seq;
    ros::Time stamp = ros::Time::now();
    pose_msg_.header.seq = seq;
    pose_msg_.header.stamp = stamp;
    pose_msg_.header.frame_id = global_frame_;
    pose_msg_.pose.pose = new_pose_;
    // double eye[36]{};
    // for(int i = 0; i < 36; i += 7)
        // pose_msg_.pose.covariance[i] = 1.0;
    // pose_msg_.pose.covariance = eye;
    pose_pub_.publish(pose_msg_);
    ros::Duration(0.01).sleep();
}

void PoseFilter::broadcastFrame(){
    static tf::TransformBroadcaster br{};
	tf::Transform transform{};
    tf::Transform new_transform{};
    tf::Quaternion quat(0.0, 0.0, 0.0, 1.0);
    tf::Quaternion new_quat;

    // 子フレームから親フレームの順でpublishするべき
    // odom -> base_link
	new_transform.setOrigin(tf::Vector3(new_pose_.position.x, new_pose_.position.y, new_pose_.position.z));
    quaternionMsgToTF(new_pose_.orientation, new_quat);
	new_transform.setRotation(new_quat);
	br.sendTransform(tf::StampedTransform(new_transform, ros::Time::now(), "odom", base_frame_));

    // map -> odom
    transform.setRotation(quat);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame_, "odom"));
}
