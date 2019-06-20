// Standard
#include <string>

//ROS
#include <ros/ros.h>
#include "std_msgs/String.h"

// GPIO
#include <pigpio.h>



class MotorControl
{
    ros::NodeHandle nh_;
    ros::Subscriber motor_a_sub;
    ros::Subscriber motor_b_sub;
    std::string robot_id;

    int enA1 = 12;
    int enB1 = 13;
    int enA2 = 1;
    int enB2 = 6;

    public:
    MotorControl()

    {
        ros::NodeHandle _nh("~");

        _nh.getParam("robot_id", robot_id);
        ROS_INFO_STREAM("Robot ID: '" << robot_id);

        startGpio();

        motor_a_sub = _nh.subscribe("/digital_futures/"+robot_id+"/wheel_a", 1, &MotorControl::setMotorA, this);
        motor_b_sub = _nh.subscribe("/digital_futures/"+robot_id+"/wheel_b", 1, &MotorControl::setMotorB, this);

    }
    void startGpio(){
        // Initialize pigpio
        gpioInitialise();
        // Setup Pins
        gpioSetMode(enA1, PI_OUTPUT);
        gpioSetMode(enB1, PI_OUTPUT);
        gpioSetMode(enA2, PI_OUTPUT);
        gpioSetMode(enB2, PI_OUTPUT);
        sleep(1);
    }
    void setMotorA(const std_msgs::String::ConstPtr& msg){
        std::string speedString= msg->data.c_str();
        int speedInt = std::stoi(speedString);
        if(speedInt > 0){
            gpioWrite(enA1, 0);
            gpioPWM(enA2, speedInt);
        }else{
            gpioWrite(enA2, 0);
            gpioPWM(enA1, speedInt*-1);
        }
    }
    void setMotorB(const std_msgs::String::ConstPtr& msg){
        std::string speedString= msg->data.c_str();
        int speedInt = std::stoi(speedString);
        if(speedInt > 0){
            gpioWrite(enB1, 0);
            gpioPWM(enB2, speedInt);
        }else{
            gpioWrite(enB2, 0);
            gpioPWM(enB1, speedInt*-1);
        }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_control");
  MotorControl mC;
  ros::spin();
  return 0;
}