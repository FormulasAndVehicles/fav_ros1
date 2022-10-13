#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

namespace gazebo {
static constexpr auto kDefaultPubRate = 50.0;
static constexpr auto kDefaultPressureTopic = "pressure";
static constexpr auto kDefaultPressureNoise = 100.0;

class BarometerPlugin : public ModelPlugin {
 public:
  BarometerPlugin();
  ~BarometerPlugin() override;

 protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
  void ParseSdf(sdf::ElementPtr _sdf);

 private:
  struct {
    std::string robotNamespace;
    std::string link{"base_link"};
    ignition::math::Vector3d position{0.0, 0.0, 0.0};
    double publish_rate{50.0};
    std::string topic{"pressure"};
    double noise{100.0};
  } sdf_params_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  ros::NodeHandle *node_handle_;
  ros::Publisher pressure_pub_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  common::Time last_pub_time_;
  common::Time last_time_;

  double baro_rnd_y2_;
  bool baro_rnd_use_last_;
};
}  // namespace gazebo
