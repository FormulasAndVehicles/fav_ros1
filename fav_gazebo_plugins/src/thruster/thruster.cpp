#include "thruster.hpp"

#include "common.hpp"

namespace gazebo {
namespace direction {
static constexpr int CCW = -1;
static constexpr int CW = 1;
}  // namespace direction
GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin);
void ThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  ParseSdf(_sdf);
  model_ = _model;
  link_ = model_->GetLink(sdf_params_.link);
  parent_link_ = model_->GetLink(sdf_params_.parent_link);
  joint_ = model_->GetJoint(sdf_params_.joint);
  world_ = model_->GetWorld();

  rotor_velocity_filter_ = std::make_unique<FirstOrderFilter<double>>(
      sdf_params_.timeconstant_up, sdf_params_.timeconstant_down,
      rotor_velocity_setpoint_);

  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  std::string ns = sdf_params_.robotNamespace + "thruster_" +
                   std::to_string(sdf_params_.thruster_number);
  std::string topic_name = "~/" + sdf_params_.thrust_base_topic;
  node_->Init(ns);
  thrust_sub_ = node_->Subscribe(topic_name, &ThrusterPlugin::OnThrust, this);
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ThrusterPlugin::OnUpdate, this, _1));
}
void ThrusterPlugin::OnUpdate(const common::UpdateInfo &_info) {
  double dt = (_info.simTime - prev_sim_time_).Double();
  prev_sim_time_ = _info.simTime;
  UpdateRotorVelocity(dt);
  UpdateForcesAndMoments();
}

void ThrusterPlugin::OnThrust(ConstAnyPtr &_msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  rotor_velocity_setpoint_ = ThrustToVelocity(_msg->double_value());
}
void ThrusterPlugin::ParseSdf(sdf::ElementPtr _sdf) {
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  AssignSdfParam(_sdf, "parent_link", sdf_params_.parent_link);
  AssignSdfParam(_sdf, "robotNamespace", sdf_params_.robotNamespace);
  AssignSdfParam(_sdf, "joint", sdf_params_.joint);
  AssignSdfParam(_sdf, "thrust_base_topic", sdf_params_.thrust_base_topic);
  AssignSdfParam(_sdf, "linear_coeff", sdf_params_.linear_coeff);
  AssignSdfParam(_sdf, "quadratic_coeff", sdf_params_.quadratic_coeff);
  AssignSdfParam(_sdf, "torque_coeff", sdf_params_.torque_coeff);
  AssignSdfParam(_sdf, "rpm_scaler", sdf_params_.rpm_scaler);
  AssignSdfParam(_sdf, "maximum_rpm", sdf_params_.maximum_rpm);
  AssignSdfParam(_sdf, "thruster_number", sdf_params_.thruster_number);
  AssignSdfParam(_sdf, "turning_direction", sdf_params_.turning_direction);
  if (sdf_params_.turning_direction == "cw") {
    turning_direction_ = direction::CW;
  } else {
    turning_direction_ = direction::CCW;
  }
  AssignSdfParam(_sdf, "propeller_direction", sdf_params_.propeller_direction);
  if (sdf_params_.propeller_direction == "cw") {
    propeller_direction_ = direction::CW;
  } else {
    propeller_direction_ = direction::CCW;
  }
  AssignSdfParam(_sdf, "timeconstant_up", sdf_params_.timeconstant_up);
  AssignSdfParam(_sdf, "timeconstant_down", sdf_params_.timeconstant_down);
}

void ThrusterPlugin::UpdateRotorVelocity(double _dt) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rotor_velocity_ =
        rotor_velocity_filter_->Update(rotor_velocity_setpoint_, _dt);
  }
  SetRotorVelocity(rotor_velocity_);
}

void ThrusterPlugin::SetRotorVelocity(double _velocity) {
  joint_->SetVelocity(0, _velocity / sdf_params_.rpm_scaler);
}

void ThrusterPlugin::UpdateForcesAndMoments() {
  double thrust;
  double tmp = std::abs(rotor_velocity_) / 2 / 3.14;
  thrust =
      tmp * tmp * sdf_params_.quadratic_coeff + tmp * sdf_params_.linear_coeff;
  if (rotor_velocity_ < 0) {
    thrust *= -1.0;
  }
  thrust *= propeller_direction_;

  // remove the effect of the propeller direction by multiplying thrust with it
  // again. The moment only depends on the turning direction.
  double moment = -propeller_direction_ * thrust * sdf_params_.torque_coeff;

  ignition::math::Vector3d force{thrust, 0, 0};
  ignition::math::Vector3d torque{moment, 0.0, 0.0};

  parent_link_->AddRelativeTorque(torque);
  link_->AddRelativeForce(force);
}

}  // namespace gazebo
