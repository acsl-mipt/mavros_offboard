#include <mavros/mavros_plugin.h>
//#include <mavros/gps_conversions.h>

#include <std_msgs/Float64MultiArray.h>

namespace mavros {
namespace offboard_plugins {

class GlobalPositionIntPlugin : public plugin::PluginBase
{
public:
	GlobalPositionIntPlugin() : PluginBase(),
		nh("~global_position_int")
  { }
  
  void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		gpi_pub = nh.advertise<std_msgs::Float64MultiArray>("all", 10);
		//gpi_utm_pub = nh.advertise<std_msgs::Float64MultiArray>("utm", 10);

  }
  
	Subscriptions get_subscriptions() {
		return {
			make_handler(&GlobalPositionIntPlugin::handle_global_position_int)
		};
  }
private:
	ros::NodeHandle nh;
	ros::Publisher gpi_pub; //, gpi_utm_pub;

	void handle_global_position_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GLOBAL_POSITION_INT &gpi)
	{
	  auto all = boost::make_shared<std_msgs::Float64MultiArray>();
	  //auto utm = boost::make_shared<std_msgs::Float64MultiArray>();
	  double lat,lon,relative_alt;
    double north, east;
    std::string zone;

	  lat = gpi.lat / 1E7,    // deg north-south
	  lon = gpi.lon / 1E7,    // deg east-west
    relative_alt = gpi.relative_alt / 1E3,   // m
    	  
	  all->data = {
	    gpi.time_boot_ms / 1E3,    // s
	    lat,
	    lon,
	    gpi.alt / 1E3,    // m
	    relative_alt,
	    gpi.vx / 1E2,     // m/s north
	    gpi.vy / 1E2,     // m/s east
	    gpi.vz / 1E2,     // m/s down
	    (gpi.hdg != UINT16_MAX) ? gpi.hdg / 1E2 : NAN    // deg
	  };
    gpi_pub.publish(all);
    
    //UTM
    /*    
    UTM::LLtoUTM(lat, lon, north, east, zone);
    
    utm->data = {
      east,
      north,
      relative_alt
    };
    gpi_utm_pub.publish(utm);
    */
	}
};

}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::offboard_plugins::GlobalPositionIntPlugin, mavros::plugin::PluginBase)
