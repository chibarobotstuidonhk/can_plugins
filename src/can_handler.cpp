/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_plugins/Frame.h>
#include "can_utils.hpp"

#include <stdint.h>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace can_plugins{
  class CanHandler : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      void betamotor4CmdSwingCallback(const std_msgs::Float64::ConstPtr& msg);    

      void solenoidOrderCallback(const std_msgs::UInt8::ConstPtr& msg);

      void canRxCallback(const can_plugins::Frame::ConstPtr &msg);

      void TestTxCallback(const std_msgs::UInt8::ConstPtr &msg);

      template<typename T>
        void sendData(const uint16_t id, const T data);

      ros::NodeHandle _nh;
      ros::NodeHandle pnh;
      ros::Publisher _can_tx_pub;
      ros::Subscriber _can_rx_sub;

      ros::Publisher  _base_odom_x_pub;
      ros::Publisher  _base_odom_y_pub;
      ros::Publisher  _base_odom_yaw_pub;

      ros::Publisher _limit_switch_pub;

      ros::Publisher  _test_pub;
      ros::Subscriber _test_sub;

      ros::Subscriber _solenoid_order_sub;

      static constexpr uint16_t id_baseOdomX              = 0x205;
      static constexpr uint16_t id_baseOdomY              = 0x206;
      static constexpr uint16_t id_baseOdomYaw            = 0x207;
      static constexpr uint16_t id_solenoid            = 0x100;
      static constexpr uint16_t id_test_tx=0x4d8;
      static constexpr uint16_t id_test_rx=0x4db;

      static constexpr uint16_t id_limit = 0x7fd;
      static constexpr uint16_t id_photo = 0x777;
  };

  void CanHandler::onInit(){
    _nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    _can_tx_pub				    = _nh.advertise<can_plugins::Frame>("can_tx", 1000);

    _base_odom_x_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/x", 1);
    _base_odom_y_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/y", 1);
    _base_odom_yaw_pub		    = _nh.advertise<std_msgs::Float32>("base/odom/yaw", 1);

    _limit_switch_pub = _nh.advertise<std_msgs::UInt8>("limit_switch",10);

    _test_pub = _nh.advertise<std_msgs::UInt8>("test_rx",1000);
    _test_sub = _nh.subscribe<std_msgs::UInt8>("test_tx",1000,&CanHandler::TestTxCallback,this);

    _can_rx_sub				    = _nh.subscribe<can_plugins::Frame>("can_rx", 1000, &CanHandler::canRxCallback, this);

    NODELET_INFO("can_handler has started.");
  }

  void CanHandler::TestTxCallback(const std_msgs::UInt8::ConstPtr &msg)
  {
    _can_tx_pub.publish(get_frame(id_test_tx, msg->data));
  }

  void CanHandler::canRxCallback(const can_plugins::Frame::ConstPtr &msg)
  {
    std_msgs::Float32 _base_odom_x_msg;
    std_msgs::Float32 _base_odom_y_msg;
    std_msgs::Float32 _base_odom_yaw_msg;

    std_msgs::UInt8 _test_msg;
    std_msgs::UInt8 _limit_switch_msg;

    switch(msg->id)
    {
      case id_baseOdomX:
        can_unpack(msg->data, _base_odom_x_msg.data);
        _base_odom_x_pub.publish(_base_odom_x_msg);
        break;

      case id_baseOdomY:
        can_unpack(msg->data, _base_odom_y_msg.data);
        _base_odom_y_pub.publish(_base_odom_y_msg);
        break;

      case id_baseOdomYaw:
        can_unpack(msg->data, _base_odom_yaw_msg.data);
        _base_odom_yaw_pub.publish(_base_odom_yaw_msg);
        break;

      case id_test_rx:
        can_unpack(msg->data, _test_msg.data);
        _test_pub.publish(_test_msg);
        break;

      case id_limit:
        can_unpack(msg->data,_limit_switch_msg.data);
        _limit_switch_pub.publish(_limit_switch_msg);
        break;

      case id_photo:
        can_unpack(msg->data,_limit_switch_msg.data);
        _limit_switch_pub.publish(_limit_switch_msg);
        break;
      default:
        break;
    }
  }
}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(can_plugins::CanHandler, nodelet::Nodelet);
