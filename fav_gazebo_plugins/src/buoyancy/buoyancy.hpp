#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

namespace gazebo {
class BuoyancyPlugin : public ModelPlugin {
 protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
  void ParseSdf(sdf::ElementPtr _sdf);

 private:
  void UpdateForcesAndMoments();
  struct SdfParams {
    std::string link{"base_link"};
    double additional_buoyancy_force{0.0};
    double relative_compensation{1.0};
    ignition::math::Vector3d origin{0.0, 0.0, 0.0};
    double height_scale_limit{0.1};
  } sdf_params_;
  ignition::math::Matrix3d damping_linear_matrix_;
  ignition::math::Matrix3d damping_angular_matrix_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
};
}  // namespace gazebo
