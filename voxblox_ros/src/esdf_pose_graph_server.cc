#include <voxblox_ros/conversions.h>

#include "voxblox_ros/esdf_pose_graph_server.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

EsdfPoseGraphServer::EsdfPoseGraphServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : EsdfPoseGraphServer(nh, nh_private, getEsdfMapConfigFromRosParam(nh_private),
                 getEsdfIntegratorConfigFromRosParam(nh_private),
                 getTsdfMapConfigFromRosParam(nh_private),
                 getTsdfIntegratorConfigFromRosParam(nh_private),
                 getMeshIntegratorConfigFromRosParam(nh_private)) {}

EsdfPoseGraphServer::EsdfPoseGraphServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const EsdfMap::Config& esdf_config,
                       const EsdfIntegrator::Config& esdf_integrator_config,
                       const TsdfMap::Config& tsdf_config,
                       const TsdfIntegratorBase::Config& tsdf_integrator_config,
                       const MeshIntegratorConfig& mesh_config)
    : TsdfPoseGraphServer(nh, nh_private, tsdf_config, tsdf_integrator_config,
                 mesh_config),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0) {
  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  setupRos();
}

void EsdfPoseGraphServer::setupRos() {
  // Set up publisher.
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);
  traversable_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "traversable", 1, true);

  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 1,
                                        &EsdfPoseGraphServer::esdfMapCallback, this);

  // Set up service
  query_points_gradient_srv_ = nh_private_.advertiseService("queryGradientPoints",
                            &EsdfPoseGraphServer::queryGradientPointsCallback, this);

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
  nh_private_.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);

  // Special output for traversable voxels. Publishes all voxels with distance
  // at least traversibility radius.
  nh_private_.param("publish_traversable", publish_traversable_,
                    publish_traversable_);
  nh_private_.param("traversability_radius", traversability_radius_,
                    traversability_radius_);

  double update_esdf_every_n_sec = 1.0;
  nh_private_.param("update_esdf_every_n_sec", update_esdf_every_n_sec,
                    update_esdf_every_n_sec);

  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ =
        nh_private_.createTimer(ros::Duration(update_esdf_every_n_sec),
                                &EsdfPoseGraphServer::updateEsdfEvent, this);
  }
}

void EsdfPoseGraphServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_pointcloud_pub_.publish(pointcloud);
}

void EsdfPoseGraphServer::publishSlices() {
  TsdfPoseGraphServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_slice_pub_.publish(pointcloud);
}

bool EsdfPoseGraphServer::generateEsdfCallback(
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  const bool clear_esdf = true;
  if (clear_esdf) {
    esdf_integrator_->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
  }
  publishAllUpdatedEsdfVoxels();
  publishSlices();
  return true;
}

bool EsdfPoseGraphServer::queryGradientPointsCallback(
        radiation_srvs::QueryPeaks::Request& req,
        radiation_srvs::QueryPeaks::Response& res){
    float distance = req.distance;
    std::vector<Eigen::Vector3d> queryPts;
    for(auto point : req.querypoints){
        queryPts.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    std::vector<int> querySuccess;
    std::vector<Eigen::Vector3d> waypoints;
    for(auto qPt : queryPts){
        Eigen::Vector3d qPt_temp;
        qPt_temp = qPt;
        Eigen::Vector3d vec(0.0, 0.0, 0.0);
        double coefficient = 0.1;
        if(esdf_map_->isObserved(qPt)){
            while(vec.norm() < distance){
                Eigen::Vector3d gradient(100, 100, 100);
                while( (coefficient*gradient).norm() > distance ){

                    double gradientDistance;
                    if(!esdf_map_->getDistanceAndGradientAtPosition(qPt, &gradientDistance, &gradient)){
                        ROS_ERROR("Point not in ESDF map...");
                        break;
                    }
                    if( (coefficient*gradient).norm() > distance ){
                        coefficient = coefficient/2;
                    }
                }
                vec += gradient;
            }
            Eigen::Vector3d waypoint = qPt + vec; // point which should be visited.
            waypoints.push_back(waypoint);
            querySuccess.push_back(1);
        }
        else{
            // if the query point is not in the ESDF map you do not push back
            // any waypoint.
            querySuccess.push_back(0);
        }
    }

    std::vector<geometry_msgs::Point> pointsForMessage;
    for(auto waypoint : waypoints){
        geometry_msgs::Point pt;
        pt.x = waypoint[0];
        pt.y = waypoint[1];
        pt.z = waypoint[2];
        pointsForMessage.push_back(pt);
    }
    res.querySuccess = querySuccess;
    res.waypoints = pointsForMessage;
    res.success = true;
    return true;
}

void EsdfPoseGraphServer::updateEsdfEvent(const ros::TimerEvent& /*event*/) {
  updateEsdf();
}

void EsdfPoseGraphServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfPoseGraphServer::publishPointclouds();
}

void EsdfPoseGraphServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                    traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  traversable_pub_.publish(pointcloud);
}

void EsdfPoseGraphServer::publishMap(bool reset_remote_map) {
  if (!publish_esdf_map_) {
    return;
  }

  int subscribers = this->esdf_map_pub_.getNumSubscribers();
  if (subscribers > 0) {
    if (num_subscribers_esdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->esdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_esdf_map_ = subscribers;
  TsdfPoseGraphServer::publishMap();
}

bool EsdfPoseGraphServer::saveMap(const std::string& file_path) {
  // Output TSDF map first, then ESDF.
  const bool success = TsdfPoseGraphServer::saveMap(file_path);

  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool EsdfPoseGraphServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfPoseGraphServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

void EsdfPoseGraphServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

void EsdfPoseGraphServer::updateEsdfBatch(bool full_euclidean) {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    esdf_integrator_->setFullEuclidean(full_euclidean);
    esdf_integrator_->updateFromTsdfLayerBatch();
  }
}

float EsdfPoseGraphServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void EsdfPoseGraphServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float EsdfPoseGraphServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void EsdfPoseGraphServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

void EsdfPoseGraphServer::newPoseCallback(const Transformation& T_G_C) {
  if (clear_sphere_for_planning_) {
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  block_remove_timer.Stop();
}

void EsdfPoseGraphServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO_ONCE("Got an ESDF map from ROS topic!");
    publishPointclouds();
  }
}

void EsdfPoseGraphServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfPoseGraphServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

}  // namespace voxblox
