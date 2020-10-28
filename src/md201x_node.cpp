/*
 * md201x_node.cpp
 *
 *  Created on: Feb 19, 2020
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include "can_utils.hpp"
#include <functional>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace can_plugins{

    class Md201xNode : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

    private:
        void motorCmdCallback(const std_msgs::UInt8::ConstPtr &msg);
        void motorCmdValCallback(const std_msgs::Float64::ConstPtr &msg);
        void motorCmdSwingCallback(const std_msgs::Float64::ConstPtr &msg);

        void canRxCallback(const can_plugins::Frame::ConstPtr &msg);

        ros::NodeHandle _nh;
        ros::NodeHandle _private_nh;
        ros::Publisher _can_tx_pub;
        ros::Subscriber _can_rx_sub;

        ros::Publisher _motor_status_pub;
        ros::Subscriber _motor_cmd_sub;
        ros::Subscriber _motor_cmd_val_sub;
        ros::Subscriber _motor_cmd_swing_sub;

        std::string bid;
        uint16_t id_motor_cmd;
        uint16_t id_motor_cmd_val;
        uint16_t id_motor_cmd_swing;
        uint16_t id_motor_status;

        bool _swing;

        std::string name;

        //for periodic publish feature
        double _ctrl_freq;
        ros::Timer timer;
        void motorCmdValCallbackPeriodic(const std_msgs::Float64::ConstPtr &msg);
        float _cmd_val=0.0;
    };

    void Md201xNode::motorCmdCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        _can_tx_pub.publish(get_frame(id_motor_cmd, msg->data));
    }

    void Md201xNode::motorCmdValCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        _can_tx_pub.publish(get_frame(id_motor_cmd_val, (float)msg->data));
    }

    void Md201xNode::motorCmdSwingCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        _can_tx_pub.publish(get_frame(id_motor_cmd_swing, (float)msg->data));
    }

    void Md201xNode::canRxCallback(const can_plugins::Frame::ConstPtr &msg)
    {
        if (msg->id == id_motor_status)
        {
            std_msgs::UInt8 _motor_status_msg;
            can_unpack(msg->data, _motor_status_msg.data);
            _motor_status_pub.publish(_motor_status_msg);
        }
    }

    void Md201xNode::motorCmdValCallbackPeriodic(const std_msgs::Float64::ConstPtr &msg)
    {
        _cmd_val = (float)msg->data;
    }

    void Md201xNode::onInit()
    {
        _nh = getMTNodeHandle();
        _private_nh = getMTPrivateNodeHandle();

        NODELET_INFO("md201x node has started.");
        
        this->name = this->getName();

        _private_nh.getParam("bid", this->bid);

        this->id_motor_cmd = std::strtol(this->bid.c_str(), NULL, 16);
        this->id_motor_cmd_val = this->id_motor_cmd + 1;
        this->id_motor_status = this->id_motor_cmd + 3;
        this->id_motor_cmd_swing = this->id_motor_cmd + 2;

        _can_tx_pub = _nh.advertise<can_plugins::Frame>("can_tx", 10);
        _can_rx_sub = _nh.subscribe<can_plugins::Frame>("can_rx", 10, &Md201xNode::canRxCallback, this);

        _motor_status_pub = _nh.advertise<std_msgs::UInt8>(name + "_status", 10);
        _motor_cmd_sub = _nh.subscribe<std_msgs::UInt8>(name + "_cmd", 10, &Md201xNode::motorCmdCallback, this);

        if(_private_nh.getParam("ctrl_freq", _ctrl_freq))
        {
            _motor_cmd_val_sub = _nh.subscribe<std_msgs::Float64>(name + "_cmd_val", 10, &Md201xNode::motorCmdValCallbackPeriodic, this);
            timer = _nh.createTimer(ros::Duration(1.0/_ctrl_freq),[&](const ros::TimerEvent &){
                _can_tx_pub.publish(get_frame(id_motor_cmd_val, _cmd_val));
            });
        }
        else
        {
            _motor_cmd_val_sub = _nh.subscribe<std_msgs::Float64>(name + "_cmd_val", 10, &Md201xNode::motorCmdValCallback, this);
        }

        if(_private_nh.getParam("swing",_swing))
        {
            NODELET_INFO("swing mode");
            _motor_cmd_swing_sub = _nh.subscribe<std_msgs::Float64>(name + "_cmd_swing", 10,&Md201xNode::motorCmdSwingCallback,this);
        }
    }
}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(can_plugins::Md201xNode, nodelet::Nodelet);