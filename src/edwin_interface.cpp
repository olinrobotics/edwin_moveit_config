#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "utils.h"
#include "edwin_interface.h"

const std::string joints[] = {"wrist","hand","elbow","shoulder","waist"};

EdwinInterface::EdwinInterface(ros::NodeHandle nh):nh(nh){
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

	pub = nh.advertise<std_msgs::String>("arm_cmd", 10, false);
	sub = nh.subscribe("joint_states", 10, &EdwinInterface::joint_cb, this);
}

ros::Time EdwinInterface::get_time(){
	return ros::Time::now();
}

void EdwinInterface::joint_cb(const sensor_msgs::JointStateConstPtr& msg){
	joint_state_msg = *msg;
	for(int i=0; i<N_JOINTS; ++i){
		pos[i] = joint_state_msg.position[i];
	}
}
void EdwinInterface::read(const ros::Time& time, const ros::Duration& period){
	// info from st-r17, i.e. current joint states
	//for(int i=0; i<N_JOINTS; ++i){
	//	cmd_msg.data = "get_joint_states"; 
	//	pub.publish(cmd_msg);
	//}
	std::vector<float> loc = st.where();
	loc[1] /= 2;
	loc[4] *= 3;
	loc[4] -= loc[3];

	loc[2] = -loc[2]; // flip direction
	loc[4] = -loc[4];

	for(float& l : loc){ // possibly illegal?
		l = l/100*M_PI/180;
	}
	for(int i=0; i<N_JOINTS;++i){
		pos[i] = loc[i];
	}
}

void EdwinInterface::write(const ros::Time& time, const ros::Duration& period){
	// info to st-r17, i.e. desired joint states
	for(int i=0; i<N_JOINTS; ++i){
		// DEBUGGING:
		// std::cout << (i==0?"":", ") << cmd[i];
		float dp = cmd[i] - pos[i]; // target-position
		dp = dp>0?dp:-dp;
		if(dp > d2r(1)){ // more than 1 degrees different
			std::stringstream ss;
			int k = round(r2d(posr(cmd[i])*10));
			ss << "data: rotate_" << joints[i] << ":: " << k;
			cmd_msg.data = ss.str();
			pub.publish(cmd_msg);	
		}
	}
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

		if(cnt % 10 == 0) // read every 20Hz/10 = 1 Hz
			edwin.read(now,period);

		cm.update(now, period);

		if (cnt % 30 == 0)
			edwin.write(now,period);

		++cnt;
		r.sleep();
	}
	return 0;
}
