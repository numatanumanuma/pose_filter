#include "pose_filter/pose_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_filter_node");
  //ノード名の初期化

  PoseFilter a;

  ros::spin();

  return 0;
}