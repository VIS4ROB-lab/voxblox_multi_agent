#ifndef VOXBLOX_ROS_ESDF_SERVER_H_
#define VOXBLOX_ROS_ESDF_SERVER_H_

#include <memory>
#include <string>

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_msgs/Layer.h>

#include <radiation_srvs/QueryPeaks.h>

#include "voxblox_ros/tsdf_pose_graph_server.h"

namespace voxblox {

class EsdfPoseGraphServer : public TsdfPoseGraphServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EsdfPoseGraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  EsdfPoseGraphServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const EsdfMap::Config& esdf_config,
             const EsdfIntegrator::Config& esdf_integrator_config,
             const TsdfMap::Config& tsdf_config,
             const TsdfIntegratorBase::Config& tsdf_integrator_config,
             const MeshIntegratorConfig& mesh_config);
  virtual ~EsdfPoseGraphServer() {}

  bool generateEsdfCallback(std_srvs::Empty::Request& request,     // NOLINT
                            std_srvs::Empty::Response& response);  // NOLINT

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();
  void publishTraversable();

  virtual void publishPointclouds();
  virtual void newPoseCallback(const Transformation& T_G_C);
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  void updateEsdfEvent(const ros::TimerEvent& event);

  /// Call this to update the ESDF based on latest state of the TSDF map,
  /// considering only the newly updated parts of the TSDF map (checked with
  /// the ESDF updated bit in Update::Status).
  void updateEsdf();
  /// Update the ESDF all at once; clear the existing map.
  void updateEsdfBatch(bool full_euclidean = false);

  // Overwrites the layer with what's coming from the topic!
  void esdfMapCallback(const voxblox_msgs::Layer& layer_msg);

  inline std::shared_ptr<EsdfMap> getEsdfMapPtr() { return esdf_map_; }
  inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const {
    return esdf_map_;
  }

  bool getClearSphere() const { return clear_sphere_for_planning_; }
  void setClearSphere(bool clear_sphere_for_planning) {
    clear_sphere_for_planning_ = clear_sphere_for_planning;
  }
  float getEsdfMaxDistance() const;
  void setEsdfMaxDistance(float max_distance);
  float getTraversabilityRadius() const;
  void setTraversabilityRadius(float traversability_radius);

  /**
   * These are for enabling or disabling incremental update of the ESDF. Use
   * carefully.
   */
  void disableIncrementalUpdate() { incremental_update_ = false; }
  void enableIncrementalUpdate() { incremental_update_ = true; }

  virtual void clear();

 protected:
  /// Sets up publishing and subscribing. Should only be called from
  /// constructor.
  void setupRos();

  /// send multiple query points. those points are converted to waypoints accessible
  /// for the drone, i.e. the points are shifted distance d away from the occupied
  /// space into the free space.
  bool queryGradientPointsCallback(radiation_srvs::QueryPeaks::Request& req,
                                   radiation_srvs::QueryPeaks::Response& res);

  /// Publish markers for visualization.
  ros::Publisher esdf_pointcloud_pub_;
  ros::Publisher esdf_slice_pub_;
  ros::Publisher traversable_pub_;

  /// Publish the complete map for other nodes to consume.
  ros::Publisher esdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  ros::Subscriber esdf_map_sub_;

  /// Services.
  ros::ServiceServer generate_esdf_srv_;

  // service for querying points (e.g. radiation peaks) and a corresponding distance
  // such that the resulting points are distance [m] away from the object the
  // peaks were detected on.
  ros::ServiceServer query_points_gradient_srv_;

  /// Timers.
  ros::Timer update_esdf_timer_;

  bool clear_sphere_for_planning_;
  bool publish_esdf_map_;
  bool publish_traversable_;
  float traversability_radius_;
  bool incremental_update_;
  int num_subscribers_esdf_map_;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ESDF_SERVER_H_
