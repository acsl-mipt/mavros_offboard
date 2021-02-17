#include <mavros/mavros_plugin.h>

#include <std_msgs/Float32MultiArray.h>

namespace mavros {
namespace offboard_plugins {

class LocalPositionNedPlugin : public plugin::PluginBase
{
public:
  LocalPositionNedPlugin() : PluginBase(),
    nh("~local_position_ned")
  { }

  void initialize(UAS &uas_)
  {
    PluginBase::initialize(uas_);

    lpn_vel_sub = nh.subscribe("set_target/vel", 10, &LocalPositionNedPlugin::vel_cb, this);
    accel_sub = nh.subscribe("set_target/accel", 10, &LocalPositionNedPlugin::accel_cb, this);
    pos_sub = nh.subscribe("set_target/pos", 10, &LocalPositionNedPlugin::pos_cb, this);

    vxy_pz_sub = nh.subscribe("set_target/vxy_pz", 10, &LocalPositionNedPlugin::vxy_pz_cb, this);

    //lpn_pub = nh.advertise<std_msgs::Float32MultiArray>("all", 10);
  }

  Subscriptions get_subscriptions()
  {
    return {
      //make_handler(&LocalPositionNedPlugin::handle_local_position_ned)
    };
  }


private:
  ros::NodeHandle nh;
  ros::Subscriber lpn_vel_sub,accel_sub,pos_sub,vxy_pz_sub;
  //ros::Publisher lpn_pub;

  /*
  void handle_local_position_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED &lpn)
  {
    auto all = boost::make_shared<std_msgs::Float32MultiArray>();

    all->data = {
      lpn.time_boot_ms / (float)1E3,    // s
      lpn.x, lpn.y, lpn.z,
      lpn.vx, lpn.vy, lpn.vz
    };

    lpn_pub.publish(all);
  }
  */
  void vel_cb(const std_msgs::Float32MultiArray::ConstPtr &marr) {
    uint16_t mask = (1 << 10) | (7 << 6) | (7 << 0); // 0b0000010111000111

    std::vector<float> v = marr->data;
    if (v.size() == 4)
    {
      set_position_target_local_ned(
          mask,
          0,0,0,
          v.at(0), v.at(1), v.at(2),
          0,0,0,
          0, v.at(3)
      );
    }
  }

  void accel_cb(const std_msgs::Float32MultiArray::ConstPtr &marr) {
    uint16_t mask = (3 << 10) | 63; // 0b0000110000111111
    //uint16_t mask = (7 << 9) | 63; // 0b0000111000111111
    float yaw_rate = 0;

    std::vector<float> v = marr->data;
    if (v.size() > 2)
    {
      if (v.size() == 4)
      {
        yaw_rate = v.at(3);
        mask = mask & ~(1 << 11);
      }

      set_position_target_local_ned(
          mask,
          0,0,0,
          0,0,0,
          v.at(0), v.at(1), v.at(2),
          0, yaw_rate
      );
    }
  }

  void pos_cb(const std_msgs::Float32MultiArray::ConstPtr &marr) {
    uint16_t mask = (3 << 10) | (63 << 3); // 0b0000110111111000

    std::vector<float> v = marr->data;
    if (v.size() == 3)
    {
      set_position_target_local_ned(
          mask,
          v.at(0), v.at(1), v.at(2),
          0,0,0,
          0,0,0,
          0, 0
      );
    }
  }

  void vxy_pz_cb(const std_msgs::Float32MultiArray::ConstPtr &marr) {
    //do not ignore vz, px4 ignores vx,vy,vz if one of them ignored
    uint16_t mask = (63 << 6) | (3 << 0); // 0b0000111111000011

    std::vector<float> v = marr->data;
    if (v.size() == 3)
    {
      set_position_target_local_ned(
          mask,
          0,0, v.at(2),
          v.at(0), v.at(1), 0,
          0,0,0,
          0,0
      );
    }
  }

  void set_position_target_local_ned(
    uint16_t type_mask,
    float x,float y,float z,
    float vx,float vy,float vz,
    float afx,float afy,float afz,
    float yaw, float yaw_rate)
  {
    uint32_t time_boot_ms = 0;
    uint8_t coordinate_frame = 1; //MAV_FRAME_LOCAL_NED
    //m_uas->get_time_offset

    mavlink::common::msg::SET_POSITION_TARGET_LOCAL_NED sp;

    m_uas->msg_set_target(sp);

    sp.time_boot_ms = time_boot_ms;
    sp.coordinate_frame = coordinate_frame;
    sp.type_mask = type_mask;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    sp.afx = afx;
    sp.afy = afy;
    sp.afz = afz;

    sp.yaw = yaw;
    sp.yaw_rate = yaw_rate;

    UAS_FCU(m_uas)->send_message_ignore_drop(sp);
  }

};

}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::offboard_plugins::LocalPositionNedPlugin, mavros::plugin::PluginBase)
