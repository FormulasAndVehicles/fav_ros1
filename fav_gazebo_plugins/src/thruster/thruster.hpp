#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

namespace gazebo {
template <typename T>
class FirstOrderFilter {
 public:
  FirstOrderFilter(double _tau_up, double _tau_down, T _state)
      : tau_up_(_tau_up), tau_down_(_tau_down), state_(_state) {}
  T Update(T _state, double _dt) {
    T output;
    double alpha;
    if (_state > state_) {
      alpha = exp(-_dt / tau_up_);
    } else {
      alpha = exp(-_dt / tau_down_);
    }
    output = alpha * state_ + (1.0 - alpha) * _state;
    state_ = output;
    return output;
  }

 private:
  double tau_up_;
  double tau_down_;
  T state_;
};

class ThrusterPlugin : public ModelPlugin {
 protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
  void ParseSdf(sdf::ElementPtr _sdf);
  void OnThrust(ConstAnyPtr &_msg);

 private:
  void UpdateForcesAndMoments();
  void UpdateRotorVelocity(double _dt);
  void SetRotorVelocity(double _velocity);
  double ThrustToVelocity(double _thrust) {
    return _thrust * turning_direction_ * sdf_params_.maximum_rpm / 60.0 * 2.0 * 3.14;
  }
  struct SdfParams {
    std::string link;
    std::string parent_link{"base_link"};
    std::string robotNamespace;
    std::string joint;
    std::string thrust_base_topic{"thrust"};
    int thruster_number{0};
    std::string turning_direction{"cw"};
    std::string propeller_direction{"cw"};
    double maximum_rpm{800.0};
    double rpm_scaler{10.0};
    double torque_coeff{0.0};
    double linear_coeff{0.0};
    double quadratic_coeff{0.0};
    double timeconstant_up{0.0};
    double timeconstant_down{0.0};
  } sdf_params_;

  int turning_direction_;
  int propeller_direction_;

  std::mutex mutex_;

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;
  physics::JointPtr joint_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  common::Time prev_sim_time_;

  transport::NodePtr node_;
  transport::SubscriberPtr thrust_sub_;

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  double rotor_velocity_setpoint_{0.0};
  double rotor_velocity_{0.0};
};
}  // namespace gazebo
