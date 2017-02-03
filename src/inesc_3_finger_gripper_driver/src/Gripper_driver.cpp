#include <ros/ros.h>

#include "inesc_3_finger_gripper_driver/Stop.h"
#include "inesc_3_finger_gripper_driver/Open.h"
#include "inesc_3_finger_gripper_driver/Close.h"
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
using namespace std;



char *portname = "/dev/ttyACM0";
int fd;



int set_interface_attribs(int fd, int speed, int parity) {
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		std::cerr << "error " << errno << " from tcgetattr" << std::endl;
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
									 // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		std::cerr << "error " << errno << " from tcsetattr" << std::endl;
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block) {
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		std::cerr << "error " << errno << " from tcsetattr" << std::endl;
		return;
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
		std::cerr << "error " << errno << " setting term attributes"
				<< std::endl;
}

bool stopGripper(inesc_3_finger_gripper_driver::Stop::Request &req,
		inesc_3_finger_gripper_driver::Stop::Response &res) {
	write(fd, "M04000000\n", 9);           // send 7 character greeting
	std::cout << "stopping gripper" << std::endl;
}

bool openGripper(inesc_3_finger_gripper_driver::Open::Request &req,
		inesc_3_finger_gripper_driver::Open::Response &res) {
	write(fd, "M0400FF01\n", 9);           // send 7 character greeting
	std::cout << "opening gripper" << std::endl;
}

bool closeGripper(inesc_3_finger_gripper_driver::Close::Request &req,
		inesc_3_finger_gripper_driver::Close::Response &res) {
	write(fd, "M040000FF\n", 9);           // send 7 character greeting
				std::cout << "closing gripper" << std::endl;

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "Gripper_driver_service");

  ros::NodeHandle nh("~");


	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		std::cerr << "error " << errno << "opening " << portname << " ::"
				<< strerror(errno) << std::endl;
		return 0;
	}


  ros::ServiceServer stopService = nh.advertiseService("stopGripper", stopGripper);
  ros::ServiceServer openService = nh.advertiseService("openGripper", openGripper);
  ros::ServiceServer closeService = nh.advertiseService("closeGripper", closeGripper);


  int rate = 100;
  ros::Rate loop_rate(rate);
  double beginTime;

  ROS_INFO("========THE Gripper is started");

  while (ros::ok()) {
		set_interface_attribs(fd, B115200, 0); // set speed to 115,200 bps, 8n1 (no parity)
		set_blocking(fd, 0);

	  double secs = ros::Time::now().toSec();

    ros::spinOnce();


      double currTime = secs - beginTime;


      loop_rate.sleep();

		if (tcflush(fd, TCIOFLUSH) != 0)
					std::cerr << "flushing the buffer error" << std::endl;
  }

  return 0;
}
