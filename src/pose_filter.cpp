#include "pose_filter/pose_filter.h"
   
PoseFilter::PoseFilter(){
    ros::NodeHandle nh("~");
    nh.param("global_frame",    global_frame_,  std::string("map"));
    nh.param("base_frame",      base_frame_,    std::string("base_link"));
    nh.param("pub_topic",       pub_topic_,     std::string("amcl_pose"));
    nh.param("sub_topic",       sub_topic_,     std::string("ndt_pose"));
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_topic_, 10);
    pose_sub_ = nh.subscribe(sub_topic_, 1, &PoseFilter::msgsCallback, this);
}

void PoseFilter::msgsCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs){
    // std::cout << "Here" << std::endl;
    static tf::TransformBroadcaster br{};
    geometry_msgs::PoseWithCovarianceStamped new_msg{};
    new_msg.header = msgs->header;
    new_msg.header.frame_id = global_frame_;
    for(int i = 0; i < 36; i += 7){
        new_msg.pose.covariance[i] = 1.0;
    }

    tf::Transform tf_odom_base{};
    tf_odom_base.setOrigin(tf::Vector3(
        msgs->pose.position.x,
        msgs->pose.position.y,
        0
    ));
    tf::Quaternion quat_odom_base;

    tf::Quaternion pose_quat_before(
        msgs->pose.orientation.x,
        msgs->pose.orientation.y,
        msgs->pose.orientation.z,
        msgs->pose.orientation.w
    );

    pose_quat_before.normalize();
 
    tf::Vector3 axis(0,0,1);
    double roll, pitch, yaw;
    tf::Matrix3x3(pose_quat_before).getRPY(roll, pitch, yaw);
    tf::Quaternion pose_quat_after(axis, yaw);


    geometry_msgs::Quaternion qmsg;
    qmsg.x = pose_quat_after.getX();
    qmsg.y = pose_quat_after.getY();
    qmsg.z = pose_quat_after.getZ();
    qmsg.w = pose_quat_after.getW();
    quaternionMsgToTF(qmsg, quat_odom_base);


    new_msg.pose.pose.position.x = msgs->pose.position.x;
    new_msg.pose.pose.position.y = msgs->pose.position.y;
    new_msg.pose.pose.position.z = 0;
    new_msg.pose.pose.orientation = qmsg;
    pose_pub_.publish(new_msg);

    // quaternionMsgToTF(msgs->pose.orientation, quat_odom_base);
    tf_odom_base.setRotation(quat_odom_base);
    br.sendTransform(tf::StampedTransform(tf_odom_base, msgs->header.stamp, "odom", base_frame_));

    tf::Transform tf_map_odom{};
    tf::Quaternion i(0, 0, 0, 1);
    tf_map_odom.setRotation(i);
    br.sendTransform(tf::StampedTransform(tf_map_odom, msgs->header.stamp,  global_frame_, "odom"));
}