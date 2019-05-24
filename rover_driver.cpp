#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mini_rover/imInfo.h>
#include <serial/serial.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "std_msgs/String.h"

#define INVALID_VAL 9999

using std::string;

class RoverDriver
{
public:
  RoverDriver();
  serial::Serial s_port_adaControl; // mini_rover control
  bool openSerialPort(serial::Serial &s, const string &port, uint32_t baudrate);
  string cableControl(string cmdStr);
  ros::NodeHandle nh_;
  int A,B;
  std::string ss;
ros::Publisher serialback = nh_.advertise<std_msgs::String>("serialinf",1000);
};


RoverDriver::RoverDriver():
A(0),
B(0)
{
    //make sure the serial port is closed
  if (s_port_adaControl.isOpen())
    s_port_adaControl.close();

  //driver initialization and connection
  //obtain serial port and baudrate from the parameter server
  std::string serial_port_;
  int baudrate_;
  string tmp_str = "Ali";
  string tmp_str1;
  //get the control serial 
  if (!nh_.getParam("/rover_params/s_port_adaControl",serial_port_))
  {
    serial_port_ = "/dev/ttyUSB0";
  }
  if (!nh_.getParam("/rover_params/baudrate_adaControl",baudrate_))
  {
    baudrate_ = 115200;
  }
  //only proceed if connection is successful with the COM port
  if (openSerialPort(s_port_adaControl, serial_port_,baudrate_))
  {
    ROS_INFO("rover control serial port open, port:%s, baudrate:%d",serial_port_.c_str(),baudrate_);
  }
  else
  {
    ROS_ERROR("rover control serial port failed to open, exit!");
    ros::shutdown();
  }

//std::string ss;
//std_msgs::String msg;
//ss = cableControl(tmp_str);
//msg.data = ss;
ss = cableControl(tmp_str);
}


string RoverDriver::cableControl(string cmdStr)
{
    ROS_INFO("ada_crtl_str:%s",cmdStr.c_str());  
    std::string s;
    //send to rover control
    if (s_port_adaControl.isOpen())
    {
      int n_byte = s_port_adaControl.write(cmdStr.c_str());
      s = s_port_adaControl.readline();
     //unsigned char* ss;
     //std::stringstream ss;
     //s_port_adaControl.read(ss, 255);
     //sss = (reinterpret_cast<char*>(ss));
     
    }
    return s;
}


bool RoverDriver::openSerialPort(serial::Serial &s, const string &port, uint32_t baudrate)
{
  //make sure port is not empty
  if (!port.empty())
  {
    s.setPort(port);
  }
  else
  {
    ROS_ERROR("serial port is not assigned");
    return false;
  }

  //set baudrate
  s.setBaudrate(baudrate);
  //set timeouts
  s.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);

  //try to open the port
  if (!s.isOpen())
  {
    ROS_INFO("opening serial port on port %s, and baud rate %d",port.c_str(),baudrate);
    try{ 
        s.open();
    }
    catch (serial::IOException e){
        ROS_ERROR("serial::IOException:%s",e.what());
    }
  }

  if (!s.isOpen())
  {
    ROS_ERROR("serial port open failed !");
    return false;
  }else
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover_driver");
  RoverDriver rover_driver;

  ros::Rate loop_rate(0.5);
  //ros::spin();
  while(ros::ok())
  {

    
    std_msgs::String msg;
    msg.data = rover_driver.ss;
    rover_driver.serialback.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
