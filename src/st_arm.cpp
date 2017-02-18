#include "st_arm.h"

STArm::STArm(const std::string port){

	// open serial port
	ser.Open(port);
	ser.SetBaudRate(SerialStreamBuf::BAUD_19200); //only supports baud rate = 19200 for now
	ser.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	ser.SetParity(SerialStreamBuf::PARITY_NONE);
	ser.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);

	initialize();
	start();
}

void STArm::write(const std::string str){
	// first, flush input 
	ser.sync();

	ser << str << '\r';
	ser.flush();

	block_on_result();
}

std::string STArm::block_on_result(){
	std::stringstream s;
	std::string str;
	//wait till ok
	
	while(str.find("OK") == std::string::npos)
		std::getline(ser,str);
		s << str;

	return s.str();
}

void STArm::initialize(){
	ser.sync();
	//sequence of operations
	roboforth();joint();start();calibrate();home();

	// perform initial calibration
}

void STArm::start(){

}
