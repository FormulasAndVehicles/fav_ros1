#include "hydrodynamics.hpp"

#include "common.hpp"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(HydrodynamicsPlugin);
void HydrodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  ParseSdf(_sdf);
  model_ = _model;
  link_ = model_->GetLink(sdf_params_.link);
  world_ = model_->GetWorld();
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HydrodynamicsPlugin::OnUpdate, this, _1));
}
void HydrodynamicsPlugin::OnUpdate(const common::UpdateInfo &) {
  UpdateForcesAndMoments();
}
void HydrodynamicsPlugin::ParseSdf(sdf::ElementPtr _sdf) {
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  AssignSdfParam(_sdf, "added_mass_linear", sdf_params_.added_mass_linear);
  AssignSdfParam(_sdf, "added_mass_angular", sdf_params_.added_mass_angular);
  AssignSdfParam(_sdf, "damping_linear", sdf_params_.damping_linear);
  damping_linear_matrix_(0, 0) = -sdf_params_.damping_linear.X();
  damping_linear_matrix_(1, 1) = -sdf_params_.damping_linear.Y();
  damping_linear_matrix_(2, 2) = -sdf_params_.damping_linear.Z();
  AssignSdfParam(_sdf, "damping_angular", sdf_params_.damping_angular);
  damping_angular_matrix_(0, 0) = -sdf_params_.damping_angular.X();
  damping_angular_matrix_(1, 1) = -sdf_params_.damping_angular.Y();
  damping_angular_matrix_(2, 2) = -sdf_params_.damping_angular.Z();
}

void HydrodynamicsPlugin::UpdateForcesAndMoments() {
  ignition::math::Vector3d v = link_->RelativeLinearVel();
  ignition::math::Vector3d w = link_->RelativeAngularVel();

  const ignition::math::Vector3d &m_linear = sdf_params_.added_mass_linear;

  ignition::math::Matrix3d corriolis_force_matrix{
      0.0, m_linear.Z() * v.Z(), -m_linear.Y() * v.Y(), -m_linear.Z() * v.Z(),
      0.0, m_linear.X() * v.X(), m_linear.Y() * v.Y(),  -m_linear.X() * v.X(),
      0.0};

  const ignition::math::Vector3d &m_angular = sdf_params_.added_mass_angular;
  ignition::math::Matrix3d corriolis_moment_matrix{0.0,
                                                   m_angular.Z() * w.Z(),
                                                   -m_angular.Y() * w.Y(),
                                                   -m_angular.Z() * w.Z(),
                                                   0.0,
                                                   m_angular.X() * w.X(),
                                                   m_angular.Y() * w.Y(),
                                                   -m_angular.X() * w.X(),
                                                   0.0};
  auto damping_force = damping_linear_matrix_ * v;
  auto damping_moment = damping_angular_matrix_ * w;

  auto corriolis_force = corriolis_force_matrix * w;
  auto corriolis_moment =
      corriolis_force_matrix * v + corriolis_moment_matrix * w;

  auto total_force = damping_force + corriolis_force;
  auto total_moment = damping_moment + corriolis_moment;

  link_->AddRelativeForce(total_force);
  link_->AddRelativeTorque(total_moment);
}

}  // namespace gazebo
