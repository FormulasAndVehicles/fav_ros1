#include "buoyancy.hpp"

#include "common.hpp"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin);
void BuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  ParseSdf(_sdf);
  model_ = _model;
  link_ = model_->GetLink(sdf_params_.link);
  world_ = model_->GetWorld();
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BuoyancyPlugin::OnUpdate, this, _1));
}
void BuoyancyPlugin::OnUpdate(const common::UpdateInfo &) {
  UpdateForcesAndMoments();
}
void BuoyancyPlugin::ParseSdf(sdf::ElementPtr _sdf) {
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  AssignSdfParam(_sdf, "additional_buoyancy_force",
                 sdf_params_.additional_buoyancy_force);
  AssignSdfParam(_sdf, "relative_compensation",
                 sdf_params_.relative_compensation);
  AssignSdfParam(_sdf, "origin", sdf_params_.origin);
  AssignSdfParam(_sdf, "height_scale_limit", sdf_params_.height_scale_limit);
}

void BuoyancyPlugin::UpdateForcesAndMoments() {
  ignition::math::Pose3d pose = link_->WorldPose();
  ignition::math::Vector3d center_of_buoyancy =
      pose.Pos() + pose.Rot().RotateVector(sdf_params_.origin);
  ignition::math::Vector3d force = -sdf_params_.relative_compensation *
                                   link_->GetInertial()->Mass() *
                                   world_->Gravity();

  double scale =
      std::abs((center_of_buoyancy.Z() - sdf_params_.height_scale_limit) /
               (2 * sdf_params_.height_scale_limit));
  if (center_of_buoyancy.Z() > sdf_params_.height_scale_limit) {
    scale = 0.0;
  }
  scale = ignition::math::clamp(scale, 0.0, 1.0);
  force *= scale;

  link_->AddForceAtWorldPosition(force, center_of_buoyancy);
}

}  // namespace gazebo
