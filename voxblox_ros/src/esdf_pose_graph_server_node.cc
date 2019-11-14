#include "voxblox_ros/esdf_pose_graph_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::EsdfPoseGraphServer node(nh, nh_private);

  ros::spin();
  return 0;
}
