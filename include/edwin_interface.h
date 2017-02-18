#ifndef __EDWIN_INTERFACE_H__
#define __EDWIN_INTERFACE_H__

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <string>

#include "st_arm.h"

#define N_JOINTS 5

class EdwinInterface: public hardware_interface::RobotHW{
	private:
		STArm st;

		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;

		double cmd[N_JOINTS];
		double pos[N_JOINTS];
		double vel[N_JOINTS];
		double eff[N_JOINTS];

		ros::NodeHandle nh;

		ros::Publisher pub;
		ros::Subscriber sub;

		std_msgs::String cmd_msg;
		sensor_msgs::JointState joint_state_msg;
	public:
		EdwinInterface(ros::NodeHandle nh);
		ros::Time get_time();
		void joint_cb(const sensor_msgs::JointStateConstPtr& msg);
		virtual void read(const ros::Time& time, const ros::Duration& period);
		virtual void write(const ros::Time& time, const ros::Duration& period);
};

enum {WRIST,HAND,ELBOW,SHOULDER,WAIST};
extern const std::string joints[];// = {"wrist","hand","elbow","shoulder","waist"};

#endif
