#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mini_rover/imInfo.h>
#include <serial/serial.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>

#define INVALID_VAL 9999

using std::string;

class RoverDriver
{
public:
  RoverDriver();
  serial::Serial s_port_adaControl; // mini_rover control


private:
  bool openSerialPort(serial::Serial &s, const string &port, uint32_t baudrate);
  void cableControl(string cmdStr);
  ros::NodeHandle nh_;
  int A,B;
};


RoverDriver::RoverDriver():
{
    //make sure the serial port is closed
  if (s_port_adaControl.isOpen())
    s_port_adaControl.close();

  //driver initialization and connection
  //obtain serial port and baudrate from the parameter server
  std::string serial_port_;
  int baudrate_;
  string tmp_str = "Ali";
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

 cableControl(tmp_str);

}


void RoverDriver::cableControl(string cmdStr)
{
    ROS_INFO("ada_crtl_str:%s",cmdStr.c_str());  

    //send to rover control
    if (s_port_adaControl.isOpen())
    {
      int n_byte = s_port_adaControl.write(cmdStr.c_str());
      if (n_byte <= 0)
        {
            msgStr = "rover control Failed to write data to the serial port";
            ROS_ERROR("%s",msgStr.c_str());
        }

    }

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

  ros::Rate r(20);
  //ros::spin();
  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();
  }
}
