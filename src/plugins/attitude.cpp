#include <mavros/mavros_plugin.h>
//#include <mavros/gps_conversions.h>

#include <std_msgs/Float64MultiArray.h>

namespace mavros {
namespace offboard_plugins {

class AttitudePlugin : public plugin::PluginBase
{
public:
	AttitudePlugin() : PluginBase(),
		nh("~attitude")
  { }

  void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		all_pub = nh.advertise<std_msgs::Float64MultiArray>("all", 10);
  }

	Subscriptions get_subscriptions() {
		return {
			make_handler(&AttitudePlugin::handle_msg)
		};
  }
private:
	ros::NodeHandle nh;
	ros::Publisher all_pub;

	void handle_msg(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE &cmsg)
	{
	  auto all = boost::make_shared<std_msgs::Float64MultiArray>();

	  all->data = {
      cmsg.time_boot_ms / 1E3,    // s
      cmsg.roll,
      cmsg.pitch,
      cmsg.yaw,
      cmsg.rollspeed,
      cmsg.pitchspeed,
      cmsg.yawspeed
	  };
    all_pub.publish(all);
	}
};

}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::offboard_plugins::AttitudePlugin, mavros::plugin::PluginBase)
