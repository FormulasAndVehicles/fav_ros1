#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

namespace gazebo {
class HydrodynamicsPlugin : public ModelPlugin {
 protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
  void ParseSdf(sdf::ElementPtr _sdf);

 private:
  void UpdateForcesAndMoments();
  struct SdfParams {
    ignition::math::Vector3d added_mass_linear{0.0, 0.0, 0.0};
    ignition::math::Vector3d added_mass_angular{0.0, 0.0, 0.0};
    ignition::math::Vector3d damping_linear{0.0, 0.0, 0.0};
    ignition::math::Vector3d damping_angular{0.0, 0.0, 0.0};
    std::string link{"base_link"};
  } sdf_params_;
  ignition::math::Matrix3d damping_linear_matrix_;
  ignition::math::Matrix3d damping_angular_matrix_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
};
}  // namespace gazebo
