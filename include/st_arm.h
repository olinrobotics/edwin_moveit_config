#ifndef __ST_ARM_H__
#define __ST_ARM_H__

#include <SerialStream.h>
#include <vector>
#include <string>
#include <sstream>

using namespace LibSerial;

class STArm{
	private:
		SerialStream ser;
		void write(const std::string str);
		//
	public:
		STArm(const std::string port="/dev/ttyUSB0"); //default port

		void home();
		void initialize();
		void purge();
		void roboforth();
		void calibrate();
		void start();
		std::vector<float> where();
		void joint();

		void create_route();
		std::string block_on_result(); // necessary?

		// custom command
		void execute_command();

		void set(std::string cmd);
		void set_speed(int speed);
		void set_cartesian(bool);
		void set_decimal();
		void set_continuous();
		void set_segmented();

};

#endif
