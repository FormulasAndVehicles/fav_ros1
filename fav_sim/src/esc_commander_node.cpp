#include <fav_msgs/ThrusterSetpoint.h>
#include <gazebo_msgs/DeleteModel.h>
#include <ignition/msgs/double.pb.h>
#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>

class EscCommanderNode {
 public:
  EscCommanderNode(ros::NodeHandle *_ros_node) {
    ros_node_ = _ros_node;
    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init();
    ROS_INFO("Waiting for gazebo.");
    ros::service::waitForService("/gazebo/reset_world");
    ROS_INFO("Gazebo available. Starting...");
    for (int i = 0; i < 8; ++i) {
      std::string base_topic = "thruster_" + std::to_string(i) + "/thrust";
      std::string topic_name = "/gazebo" + ros_node_->resolveName(base_topic);
      thrust_pubs_.push_back(
          gz_node_->Advertise<gazebo::msgs::Any>(topic_name));
    }
    setpoint_topic_ = "thruster_setpoint";
    watchdog_ = ros_node_->createTimer(
        ros::Duration(0.3),
        boost::bind(&EscCommanderNode::OnWatchdogTimeout, this, _1));
    setpoint_sub_ = ros_node_->subscribe<fav_msgs::ThrusterSetpoint>(
        setpoint_topic_, 1,
        boost::bind(&EscCommanderNode::OnThrusterSetpoint, this, _1));
  }

  void Run() { ros::spin(); }

 private:
  void OnWatchdogTimeout(const ros::TimerEvent &) {
    ROS_WARN_COND(!timed_out_,
                  "'%s' timed out. Sending zero thrust until new "
                  "messages arrive.",
                  setpoint_topic_.c_str());
    timed_out_ = true;
    gazebo::msgs::Any msg;
    msg.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
    msg.set_double_value(0.0);
    for (int i = 0; i < thrust_pubs_.size(); ++i) {
      thrust_pubs_[i]->Publish(msg);
    }
  }
  void OnThrusterSetpoint(const fav_msgs::ThrusterSetpoint::ConstPtr &_msg) {
    ROS_INFO_COND(timed_out_, "Received '%s'", setpoint_topic_.c_str());
    timed_out_ = false;
    watchdog_.stop();
    watchdog_ = ros_node_->createTimer(
        ros::Duration(0.3),
        boost::bind(&EscCommanderNode::OnWatchdogTimeout, this, _1));
    for (int i = 0; i < _msg->data.size(); ++i) {
      gazebo::msgs::Any msg;
      msg.set_double_value(_msg->data[i]);
      msg.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
      thrust_pubs_[i]->Publish(msg);
    }
  }

  static EscCommanderNode instance_;
  gazebo::transport::NodePtr gz_node_;
  std::vector<gazebo::transport::PublisherPtr> thrust_pubs_;
  ros::NodeHandle *ros_node_;
  ros::Subscriber setpoint_sub_;
  ros::Timer watchdog_;
  std::string setpoint_topic_;
  bool timed_out_{false};
};

void OnShutdown(int _signal) {
  ros::NodeHandle n;
  if (_signal == SIGINT) {
    ros::ServiceClient client =
        n.serviceClient<gazebo_msgs::DeleteModel>(
            "/gazebo/delete_model");
    gazebo_msgs::DeleteModel service;
    std::string name = n.getNamespace();
    name.erase(0, 1);
    service.request.model_name = name;
    if (!client.call(service)) {
      ROS_ERROR("Failed to delete model: %s", name.c_str());
    } else {
      ROS_INFO("Model deleted: %s", name.c_str());
    }
  }
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "esc_commander", ros::init_options::NoSigintHandler);
  gazebo::client::setup(argc, argv);
  ros::NodeHandle ros_node;
  signal(SIGINT, OnShutdown);
  EscCommanderNode node(&ros_node);
  node.Run();
  gazebo::client::shutdown();
}
