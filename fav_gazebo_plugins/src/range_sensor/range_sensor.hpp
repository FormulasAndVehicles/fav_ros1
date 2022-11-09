#pragma once
#include <fav_msgs/RangeMeasurement.h>
#include <fav_msgs/RangeMeasurementArray.h>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

namespace gazebo {
static constexpr double kDefaultPubRate = 7.0;
static constexpr char kDefaultRangesTopic[] = "tag_detections";
static constexpr double kDefaultRangesNoise = 0.1;
static constexpr double kDefaultFov = 90;
static constexpr double kDefaultViewingAngle = 140;
static constexpr double kDefaultDropProb = 0.05;
static constexpr double kDefaultMaxDetectionDist = 5.0;
static constexpr double kDefaultDistDropProbExponent = 2.0;

class RangeSensorPlugin : public ModelPlugin {
 public:
  RangeSensorPlugin();
  virtual ~RangeSensorPlugin();

 protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
  void getSdfParams(sdf::ElementPtr sdf);
  fav_msgs::RangeMeasurement GetRangeMsg(
      int id, ignition::math::Vector3d sensor_to_tag);
  bool IsDetected(ignition::math::Vector3d sensor_to_tag,
                  ignition::math::Vector3d body_x_axis);
  double GetDistanceDropProp(double dist);

 private:
  std::string GetTagName(int _tag_id);
  bool GetTagPosition(int _tag_id);
  bool InitTagPositions();
  
  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  std::string ranges_topic_;
  double range_noise_std_;
  double max_fov_angle_;
  double max_viewing_angle_;
  double drop_prob_;
  double max_detection_dist_;
  double dist_drop_prob_exponent_;

  ros::NodeHandle *node_handle_;
  ros::Publisher ranges_pub_;

  double pub_rate_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;
  std::uniform_real_distribution<double> uniform_real_distribution_;

  common::Time last_pub_time_;
  common::Time last_time_;

  std::map<int, ignition::math::Vector3d> tag_positions_;

  ignition::math::Vector3d tag_axis_;

  bool initialized_;
};
}  // namespace gazebo
