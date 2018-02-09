#include <mavros/mavros_plugin.h>
//#include <mavros/gps_conversions.h>

#include <std_msgs/Float64MultiArray.h>

namespace mavros {
namespace offboard_plugins {

class GPSRawIntPlugin : public plugin::PluginBase
{
public:
	GPSRawIntPlugin() : PluginBase(),
		nh("~gps_raw_int")
  { }

  void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		all_pub = nh.advertise<std_msgs::Float64MultiArray>("all", 10);
  }

	Subscriptions get_subscriptions() {
		return {
			make_handler(&GPSRawIntPlugin::handle_gps_raw_int)
		};
  }
private:
	ros::NodeHandle nh;
	ros::Publisher all_pub;

	void handle_gps_raw_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RAW_INT &gri)
	{
	  auto all = boost::make_shared<std_msgs::Float64MultiArray>();

	  all->data = {
      gri.time_usec / 1E6,    // s
      double(gri.fix_type),
      gri.lat / 1E7,    // deg north-south
      gri.lon / 1E7,    // deg east-west
      gri.alt / 1E3,    // m
      (gri.eph != UINT16_MAX) ? gri.eph : NAN,
      (gri.epv != UINT16_MAX) ? gri.epv : NAN,
      (gri.vel != UINT16_MAX) ? gri.vel / 1E2 : NAN,     // m/s
      (gri.cog != UINT16_MAX) ? gri.cog / 1E2 : NAN,     // deg
      (gri.satellites_visible != 255) ? gri.satellites_visible : NAN,
      gri.alt_ellipsoid / 1E3,    // m
      gri.h_acc / 1E3,    // m
      gri.v_acc / 1E3,    // m
      gri.vel_acc / 1E3,    // m/s
      gri.hdg_acc / 1E5    // deg
	  };
    all_pub.publish(all);
	}
};

}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::offboard_plugins::GPSRawIntPlugin, mavros::plugin::PluginBase)
