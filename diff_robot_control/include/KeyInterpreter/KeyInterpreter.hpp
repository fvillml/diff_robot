#pragma once

#include <thread>
#include <termios.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class KeyInterpreter {
public:
    KeyInterpreter();
    ~KeyInterpreter();

    void quit();
    void interpreteKeys();

private:
    int file_descriptor_ { 0 };
    struct termios old_termios_, new_termios_;
    float linear_speed_ { 0.1 }, angular_speed_ { 0.1 };
    std::atomic_int ang_mult_ { 0 }, lin_mult_ { 0 };
    ros::NodeHandle node_handler_;
    ros::Publisher cmd_vel_publisher_;
    const char* msg_ = R"(
        ---------------------------
        Moving around:
             i    
        j    k    l

        w/s : increase/decrease linear speed by 0.5 m/s
        d/a : increase/decrease angular speed by 0.5 rad/s
        m   : show this message
        anything else : force stop
        CTRL-C to quit 
        ---------------------------)";

    void keyLoop();
    void publish();
};