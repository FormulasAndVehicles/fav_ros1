#include <fav_msgs/ThrusterSetpoint.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "fav_sim/mixer/simple_mixer.hpp"
using namespace fav_sim;
class Mixer {
 public:
  Mixer() {}
  void Run() {
    node_handle_ = new ros::NodeHandle();
    if (!GetParams()) {
      return;
    }
    setpoint_pub_ = node_handle_->advertise<fav_msgs::ThrusterSetpoint>(
        "thruster_setpoint", 1);

    watchdog_ = node_handle_->createTimer(
        ros::Duration(0.3), boost::bind(&Mixer::OnWatchdogTimeout, this, _1));

    roll_sub_ = node_handle_->subscribe<std_msgs::Float64>(
        "roll", 1, boost::bind(&Mixer::OnInput, this, _1, 0));
    pitch_sub_ = node_handle_->subscribe<std_msgs::Float64>(
        "pitch", 1, boost::bind(&Mixer::OnInput, this, _1, 1));
    yaw_sub_ = node_handle_->subscribe<std_msgs::Float64>(
        "yaw", 1, boost::bind(&Mixer::OnInput, this, _1, 2));
    thrust_sub_ = node_handle_->subscribe<std_msgs::Float64>(
        "thrust", 1, boost::bind(&Mixer::OnInput, this, _1, 3));
    vertical_thrust_sub_ = node_handle_->subscribe<std_msgs::Float64>(
        "vertical_thrust", 1, boost::bind(&Mixer::OnInput, this, _1, 4));
    lateral_thrust_sub_ = node_handle_->subscribe<std_msgs::Float64>(
        "lateral_thrust", 1, boost::bind(&Mixer::OnInput, this, _1, 5));

    ros::Rate rate{ros::Duration(0.02)};
    fav_msgs::ThrusterSetpoint msg;
    while (ros::ok()) {
      rate.sleep();
      msg.header.stamp = ros::Time::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        std::array<double, mixer::kChannels> tmp = mixer_.Mix(setpoint_);
        std::copy(tmp.begin(), tmp.end(), msg.data.begin());
      }
      if (!timed_out_) {
        setpoint_pub_.publish(msg);
      } else {
        fav_msgs::ThrusterSetpoint zero_msg;
        zero_msg.header.stamp = ros::Time::now();
        setpoint_pub_.publish(zero_msg);
      }
      ros::spinOnce();
    }
  }

  void OnWatchdogTimeout(const ros::TimerEvent &) {
    ROS_WARN_COND(!timed_out_,
                  "Timed out. Sending zero thrust until new "
                  "messages arrive.");
    timed_out_ = true;
    fav_msgs::ThrusterSetpoint msg;
    msg.header.stamp = ros::Time::now();
    setpoint_pub_.publish(msg);
  }

 private:
  void OnInput(const std_msgs::Float64::ConstPtr &_msg, int i) {
    if (i < mixer::kChannels) {
      timed_out_ = false;
      watchdog_.stop();
      watchdog_ = node_handle_->createTimer(
          ros::Duration(0.5), boost::bind(&Mixer::OnWatchdogTimeout, this, _1));
      std::lock_guard<std::mutex> lock(mutex_);
      setpoint_[i] = _msg->data;
    }
  }
  bool GetParams() {
    std::vector<double> mixer_matrix;
    if (!ros::param::get("~mixer_matrix", mixer_matrix)) {
      ROS_FATAL("Param mixer_matrix not set!");
      return false;
    }
    if (mixer_matrix.size() != mixer::kChannels * mixer::kChannels) {
      ROS_FATAL("Param ~mixer_matrix has wrong size. Is %lu, expected %d.",
                mixer_matrix.size(), mixer::kChannels * mixer::kChannels);
      return false;
    }

    std::vector<double> limit_matrix;
    if (!ros::param::get("~limit_matrix", limit_matrix)) {
      ROS_FATAL("Param limit_matrix not set!");
      return false;
    }
    if (limit_matrix.size() != mixer::kChannels * mixer::kChannels) {
      ROS_FATAL("Param ~limit_matrix has wrong size. Is %lu, expected %d.",
                limit_matrix.size(), mixer::kChannels * mixer::kChannels);
      return false;
    }

    for (int i = 0; i < mixer::kChannels; ++i) {
      mixer::Mapping mapping;
      for (int j = 0; j < mixer::kChannels; ++j) {
        mapping.scalings[j] = mixer_matrix[i * mixer::kChannels + j];
        mapping.limits[j] = limit_matrix[i * mixer::kChannels + j];
      }
      mixer_.SetMapping(i, mapping);
    }

    return true;
  }
  ros::NodeHandle *node_handle_;
  ros::Publisher setpoint_pub_;
  ros::Subscriber roll_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber thrust_sub_;
  ros::Subscriber vertical_thrust_sub_;
  ros::Subscriber lateral_thrust_sub_;
  ros::Timer watchdog_;
  bool timed_out_{true};

  std::array<double, mixer::kChannels> setpoint_;

  mixer::SimpleMixer mixer_;

  std::mutex mutex_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mixer");
  Mixer node;
  node.Run();
  return 0;
}
