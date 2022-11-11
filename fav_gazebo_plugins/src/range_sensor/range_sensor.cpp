#include "range_sensor.hpp"

#include <math.h>

#include <string>

#include "common.hpp"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(RangeSensorPlugin)

RangeSensorPlugin::RangeSensorPlugin() : ModelPlugin() {}

RangeSensorPlugin::~RangeSensorPlugin() { update_connection_->~Connection(); }

void RangeSensorPlugin::getSdfParams(sdf::ElementPtr _sdf) {
  AssignSdfParam(_sdf, "robotNamespace", sdf_params_.robotNamespace);
  AssignSdfParam(_sdf, "update_rate", sdf_params_.update_rate);
  AssignSdfParam(_sdf, "topic", sdf_params_.topic);
  AssignSdfParam(_sdf, "range_noise_std", sdf_params_.range_noise_std);
  AssignSdfParam(_sdf, "max_fov_angle", sdf_params_.max_fov_angle);
  AssignSdfParam(_sdf, "max_viewing_angle", sdf_params_.max_viewing_angle);
  AssignSdfParam(_sdf, "drop_probability", sdf_params_.drop_probability);
  AssignSdfParam(_sdf, "max_detection_distance",
                 sdf_params_.max_detection_distance);
  AssignSdfParam(_sdf, "dist_drop_probability_exp",
                 sdf_params_.dist_prob_probability_exp);
}

void RangeSensorPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  getSdfParams(sdf);
  model_ = model;
  world_ = model_->GetWorld();
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("ROS node for gazebo not initialized");
    return;
  }
  node_handle_ = new ros::NodeHandle(sdf_params_.robotNamespace);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RangeSensorPlugin::OnUpdate, this, _1));

  ranges_pub_ = node_handle_->advertise<fav_msgs::RangeMeasurementArray>(
      sdf_params_.topic, 1);

  initialized_ = false;
  tag_axis_ = ignition::math::Vector3d(0.0, 1.0, 0.0);
}

std::string RangeSensorPlugin::GetTagName(int _tag_id) {
  return "apriltag_tank_ranges::tag_" + std::to_string(_tag_id + 1);
}

bool RangeSensorPlugin::GetTagPosition(int _tag_id) {
  std::string tag_name = GetTagName(_tag_id);
  auto model = world_->ModelByName(tag_name);
  if (!model) {
    return false;
  }
  tag_positions_[_tag_id] = model->WorldPose().Pos();
  return true;
}

bool RangeSensorPlugin::InitTagPositions() {
  if (initialized_) {
    return true;
  }
  for (int i = 0; i < 4; ++i) {
    if (!GetTagPosition(i)) {
      initialized_ = false;
      return false;
    }
  }
  initialized_ = true;
  gzmsg << "Range sensor initialized!\n";
  return true;
}

void RangeSensorPlugin::OnUpdate(const common::UpdateInfo &) {
  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_pub_time_).Double();

  if (!InitTagPositions()) {
    return;
  }

  if (dt > (1.0 / sdf_params_.update_rate)) {
    fav_msgs::RangeMeasurementArray msg_array;
    msg_array.header.stamp = ros::Time::now();
    msg_array.header.frame_id = "map";

    // get world pose
    ignition::math::Vector3d pos_sensor =
        model_->GetLink("range_sensor_link")->WorldPose().Pos();
    // get orientation of body x-axis
    ignition::math::Vector3d x_unit_vector(1.0, 0.0, 0.0);
    ignition::math::Vector3d body_x_axis = model_->GetLink("range_sensor_link")
                                               ->WorldPose()
                                               .Rot()
                                               .RotateVector(x_unit_vector);

    for (auto const &tag : tag_positions_) {
      ignition::math::Vector3d dist_vec{tag.second - pos_sensor};
      if (IsDetected(dist_vec, body_x_axis)) {
        msg_array.measurements.push_back(GetRangeMsg(tag.first, dist_vec));
      }
    }

    ranges_pub_.publish(msg_array);
    last_pub_time_ = current_time;
  }
}

bool RangeSensorPlugin::IsDetected(ignition::math::Vector3d sensor_to_tag,
                                   ignition::math::Vector3d body_x_axis) {
  // tag might not be visible, determine whether tag is in fov of camera
  double fov_angle = acos(sensor_to_tag.Dot(body_x_axis) /
                          (sensor_to_tag.Length() * body_x_axis.Length()));

  // camera might not be facing tag from front
  double viewing_angle = acos(tag_axis_.Dot(body_x_axis) /
                              (tag_axis_.Length() * body_x_axis.Length()));

  bool is_visible = (fov_angle < sdf_params_.max_fov_angle) &&
                    (viewing_angle < sdf_params_.max_viewing_angle);

  // measurement might be dropped for whatever reason
  double p = uniform_real_distribution_(random_generator_);
  // additional drop probability that increases with distance to tag
  double p_dist = uniform_real_distribution_(random_generator_);
  double drop_prob_dist = GetDistanceDropProp(sensor_to_tag.Length());

  bool is_not_dropped =
      (p > sdf_params_.drop_probability) && (p_dist > drop_prob_dist);

  return is_visible && is_not_dropped;
}

fav_msgs::RangeMeasurement RangeSensorPlugin::GetRangeMsg(
    int id, ignition::math::Vector3d sensor_to_tag) {
  fav_msgs::RangeMeasurement msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.id = id;

  double distance = sensor_to_tag.Length();
  // add noise
  double noise = standard_normal_distribution_(random_generator_) *
                 sdf_params_.range_noise_std;
  msg.range = distance + noise;
  return msg;
}

double RangeSensorPlugin::GetDistanceDropProp(double dist) {
  double p = pow(dist / sdf_params_.max_detection_distance,
                 sdf_params_.dist_prob_probability_exp);
  if (p > 1.0) {
    p = 1.0;
  }
  return p;
}

}  // namespace gazebo
