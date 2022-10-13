#include "barometer.hpp"

#include "common.hpp"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin)

BarometerPlugin::BarometerPlugin()
    : ModelPlugin(), baro_rnd_y2_(0.0), baro_rnd_use_last_(false) {}

BarometerPlugin::~BarometerPlugin() { update_connection_->~Connection(); }

void BarometerPlugin::ParseSdf(sdf::ElementPtr _sdf) {
  AssignSdfParam(_sdf, "robotNamespace", sdf_params_.robotNamespace);
  AssignSdfParam(_sdf, "link", sdf_params_.link);
  AssignSdfParam(_sdf, "position", sdf_params_.position);
  AssignSdfParam(_sdf, "publish_rate", sdf_params_.publish_rate);
  AssignSdfParam(_sdf, "topic", sdf_params_.topic);
  AssignSdfParam(_sdf, "noise", sdf_params_.noise);
}

void BarometerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  ParseSdf(sdf);
  model_ = model;
  link_ = model_->GetLink(sdf_params_.link);
  world_ = model_->GetWorld();
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("Ros node for gazebo not initialized");
    return;
  }
  node_handle_ = new ros::NodeHandle(sdf_params_.robotNamespace);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BarometerPlugin::OnUpdate, this, _1));

  pressure_pub_ =
      node_handle_->advertise<sensor_msgs::FluidPressure>(sdf_params_.topic, 1);
}

void BarometerPlugin::OnUpdate(const common::UpdateInfo &) {
  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / sdf_params_.publish_rate) {
    sensor_msgs::FluidPressure msg;
    auto pose = link_->WorldPose();
    double z_height =
        pose.Pos().Z() + pose.Rot().RotateVector(sdf_params_.position).Z();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    // pressure increases by 10 kPa/m water depth.
    // pressure decreases roughly 100 Pa/8m in air.
    double msl_pressure = 101325.0;

    if (z_height > 0)
      msg.fluid_pressure = msl_pressure - z_height * 12.5;
    else
      msg.fluid_pressure = msl_pressure - z_height * 10000;

    // generate Gaussian noise sequence using polar form of Box-Muller
    // transformation
    double x1, x2, w, y1;
    if (!baro_rnd_use_last_) {
      do {
        x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
        x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
        w = x1 * x1 + x2 * x2;
      } while (w >= 1.0);
      w = sqrt((-2.0 * log(w)) / w);
      // calculate two values - the second value can be used next time because
      // it is uncorrelated
      y1 = x1 * w;
      baro_rnd_y2_ = x2 * w;
      baro_rnd_use_last_ = true;
    } else {
      // no need to repeat the calculation - use the second value from last
      // update
      y1 = baro_rnd_y2_;
      baro_rnd_use_last_ = false;
    }

    // apply noise.
    double noise = sdf_params_.noise * y1;
    msg.fluid_pressure += noise;

    pressure_pub_.publish(msg);
    last_pub_time_ = current_time;
  }
}
}  // namespace gazebo
