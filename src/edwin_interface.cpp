#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "utils.h"
#include "edwin_interface.h"
#include "Mutex.h"

const std::string joints[] = {"waist","shoulder","elbow","hand","wrist"};
Mutex mtx;

EdwinInterface::EdwinInterface(ros::NodeHandle nh):st("/dev/ttyUSB0"), nh(nh){

	// initialize arm
	st.initialize();
	st.start();
	st.set_speed(10000);
	st.home();

	char buf[64] = {};
	for(int i=0; i<N_JOINTS; ++i){
		// connect and register the joint state interface
		std::sprintf(buf, "joint_%d", i+1);
		hardware_interface::JointStateHandle state_handle(buf, &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(state_handle);

		// connect and register the joint position interface
		hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(buf), &cmd[i]);
		jnt_pos_interface.registerHandle(pos_handle);
	}
	registerInterface(&jnt_pos_interface);

	pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10, false);
	sub = nh.subscribe("arm_cmd", 10, &EdwinInterface::arm_cmd_cb, this);

	//publish joint states
	joint_state_msg.header.frame_id = "base_link";

	// urdf joint names
	const char* joints[] = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
	for(int i=0; i<N_JOINTS; ++i){
		joint_state_msg.name.push_back(joints[i]);
		joint_state_msg.position.push_back(0);
		joint_state_msg.velocity.push_back(0);
		joint_state_msg.effort.push_back(0);
	}
}

ros::Time EdwinInterface::get_time(){
	return ros::Time::now();
}

void EdwinInterface::arm_cmd_cb(const std_msgs::StringConstPtr& msg){
	std::cout << "MSG DATA ::" << msg->data << std::endl;

	// ##### MUTEX #####
	mtx.lock();
	st.write(msg->data);
	mtx.unlock();
	// #################

	// TODO : implement backwards-compatible arm-cmd
	//joint_state_msg = *msg;
	//for(int i=0; i<N_JOINTS; ++i){
	//	pos[i] = joint_state_msg.position[i];
	//}
}

void cvtJ(std::vector<double>& j){
	// convert to degrees
	j[0] *= 90./B_RATIO;
	j[1] *= 90./S_RATIO;
	j[2] *= 90./E_RATIO;
	j[3] *= 90./W_RATIO;
	j[4] *= 90./T_RATIO;
	j[4] -= j[3];

	for(std::vector<double>::iterator it = j.begin(); it!=j.end();++it){
		double& l = *it;
		l = d2r(l);
	}
}

void cvtJ_i(std::vector<double>& j){
	//invert the conversion
	for(std::vector<double>::iterator it = j.begin(); it!=j.end();++it){
		double& l = *it;
		l = r2d(l);
	}
	j[4] += j[3];

	j[0] /= 90./B_RATIO;
	j[1] /= 90./S_RATIO;
	j[2] /= 90./E_RATIO;
	j[3] /= 90./W_RATIO;
	j[4] /= 90./T_RATIO;
}

void EdwinInterface::read(const ros::Time& time){
	//alias with reference
	std::vector<double>& loc = joint_state_msg.position;
	
	// ##### MUTEX #####
	mtx.lock();	
	st.where(loc);
	mtx.unlock();

	if(loc.size() != 5){
		std::cerr << "WARNING :: INVALID LOCATION READ !!! " << std::endl;
		std::cerr << loc.size() << std::endl;
		return;
	}
	// #################

	//reformat based on scale and direction
	cvtJ(loc);

	// fill data
	for(int i=0; i<N_JOINTS;++i){
		pos[i] = loc[i];
		//std::cout << pos[i] << ',';
	}
	//std::cout << std::endl;

	//publish joint states
	joint_state_msg.header.stamp = ros::Time::now();
	pub.publish(joint_state_msg);
}

void EdwinInterface::write(const ros::Time& time){
	//st.move ... joints
	// info to st-r17, i.e. desired joint states
	std::vector<double> cmd_pos(N_JOINTS);
	for(int i=0; i<N_JOINTS; ++i){
		cmd_pos[i] = cmd[i];
	}

	cvtJ_i(cmd_pos); // invert conversion


	// ##### MUTEX #####
	mtx.lock();
	st.move(cmd_pos);
	//for(int i=0; i<N_JOINTS; ++i){
	//	// DEBUGGING:
	//	// std::cout << (i==0?"":", ") << cmd[i];
	//	
	//	float dp = cmd[i] - pos[i]; // target-position
	//	dp = dp>0?dp:-dp; // abs

	//	if(dp > d2r(1)){ // more than 1 degrees different
	//		// TODO : apply scaling factors?
	//		st.move(joints[i], cmd_pos[i]);
	//	}
	//}
	mtx.unlock();
	// #################
};


int main(int argc, char* argv[]){
	ros::init(argc,argv,"edwin_hardware");
	ros::NodeHandle nh;

	EdwinInterface edwin(nh);
	controller_manager::ControllerManager cm(&edwin, nh);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Time then = edwin.get_time();
	ros::Rate r = ros::Rate(10.0); // 10 Hz
	int cnt = 0;
	while(ros::ok()){
		ros::Time now = edwin.get_time();
		ros::Duration period = ros::Duration(now-then);
		edwin.read(now);
		cm.update(now, period);
		edwin.write(now);
		++cnt;
		r.sleep();
	}
	return 0;
}
