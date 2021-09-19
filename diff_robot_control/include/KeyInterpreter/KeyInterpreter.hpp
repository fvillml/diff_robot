#pragma once

#include <thread>
#include <termios.h>
#include <atomic>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief Class used to interprete keys from the keyboard. It converts the received key into a command for the
 * robot.
 */
class KeyInterpreter {
public:
    /**
     * @brief Construct a new KeyInterpreter.
     */
    KeyInterpreter();

    /**
     * @brief Destroy the Key Interpreter.
     */
    ~KeyInterpreter();

    /**
     * @brief Set the termios parameters back to the origin and stops the current key reading thread. Used to
     * be able to kill the thread with ctrl-c.
     */
    void quit();

    /**
     * @brief Main loop. Listen to the keyboard input and get the pressed key continuously, then interpretes
     * the key to transform it into a command to the robot.
     */
    void interpreteKeys();

private:
    /** Node handler for this class */
    ros::NodeHandle node_handler_;
    /** Velocity publisher */
    ros::Publisher cmd_vel_publisher_;
    /** File descriptor used to access to the keyboard calls */
    int file_descriptor_ { 0 };
    /** termios structure containing the old configuration of the file descriptor */
    struct termios old_termios_;
    /** termios structure containing the new configuration of the file descriptor. The new configuration
     * allows to get information from the keyboard. */
    struct termios new_termios_;
    /** Current linear speed*/
    float linear_speed_ { 0.0f };
    /** Current angular speed */
    float angular_speed_ { 0.0f };
    /** Factor to multiply the angular speed. Used to change its direction or to set it to zero. */
    std::atomic_int angular_factor_ { 0 };
    /** Factor to multiply the linear speed. Used to change its direction or to set it to zero. */
    std::atomic_int linear_factor_ { 0 };
    /** Contains the last command requested */
    std::string command_ { "" };
    /** Message with the used commands */
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

    /**
     * @brief Loop used by interpreteKeys function to get and interprete a character from the keyboard.
     */
    void keyLoop();

    /**
     * @brief publishes the angular and linear speed of the robot.
     */
    void publish();
};