#include <mavros/mavros_plugin.h>

#include <std_msgs/Float64MultiArray.h>
#include <cmath>

namespace mavros {
namespace offboard_plugins {

class HilGpsPlugin : public plugin::PluginBase
{
public:
  HilGpsPlugin() : PluginBase(),
    nh("~hil_gps")
  { }

  void initialize(UAS &uas_)
  {
    PluginBase::initialize(uas_);

    //start_time = ros::Time::now();
    set_sub = nh.subscribe("set", 10, &HilGpsPlugin::set_cb, this);
  }

  Subscriptions get_subscriptions()
  {
    return {};
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber set_sub;

  //ros::Time start_time;

  void set_cb(const std_msgs::Float64MultiArray::ConstPtr &marr) {
    std::vector<double> v = marr->data;
    if (v.size() == 9)
    {
      send_hil_gps(
        v.at(0), v.at(1), v.at(2),
        v.at(3), v.at(4),
        v.at(5), v.at(6), v.at(7),
        v.at(8)
      );
    }
  }

  void send_hil_gps(
    double lat,double lon,double alt,
    double eph,double epv,
    double vh,double vu,double cog,
    double sats)
  {
    mavlink::common::msg::HIL_GPS msg;
    //msg.time_usec = (ros::Time::now() - start_time).toNSec()/1000;
    msg.time_usec = ros::Time::now().toNSec()/1000;
    msg.fix_type = 3;
    msg.lat = round(lat * 1E7);
    msg.lon = round(lon * 1E7);
    msg.alt = round(alt * 1E3);
    msg.eph = round(eph * 1E2);
    msg.epv = round(epv * 1E2);
    msg.vel = round(vh * 1E2);

    double cog_r = cog * M_PI / 180.0;

    msg.vn = round(vh * 1E2 * cos(cog_r));
    msg.ve = round(vh * 1E2 * sin(cog_r));
    msg.vd = round(-vu * 1E2);
    msg.cog = round(cog * 1E2);
    msg.satellites_visible = sats;

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

 };

 }
 }

 #include <pluginlib/class_list_macros.h>
 PLUGINLIB_EXPORT_CLASS(mavros::offboard_plugins::HilGpsPlugin, mavros::plugin::PluginBase)
