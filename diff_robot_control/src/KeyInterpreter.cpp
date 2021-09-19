#include <unistd.h>
#include <stdio.h>
#include <termios.h>

#include <KeyInterpreter/KeyInterpreter.hpp>

// Using I J K L instead because up key will send the set of chars ^[A which
// will take time to be translated
constexpr char KEY_UP = 0x69;
constexpr char KEY_DOWN = 0x6B;
constexpr char KEY_RIGHT = 0x6C;
constexpr char KEY_LEFT = 0x6A;
constexpr char KEY_W = 0x77;
constexpr char KEY_A = 0x61;
constexpr char KEY_S = 0x73;
constexpr char KEY_D = 0x64;
constexpr char KEY_M = 0x6D;

KeyInterpreter::KeyInterpreter()
    : node_handler_() {
    cmd_vel_publisher_ = node_handler_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

KeyInterpreter::~KeyInterpreter() {
}

void KeyInterpreter::quit() {
    ROS_INFO("Calling quit");
    // Stopping the robot
    angular_factor_ = linear_factor_ = 0;
    publish();
    // sets the parameters back to old_termios_
    tcsetattr(file_descriptor_, TCSANOW, &old_termios_);
    ROS_INFO("Going to stop reading...");
    ros::shutdown();
    exit(0);
}

void KeyInterpreter::interpreteKeys() {
    // gets the parameters associated with the object referred by file_descriptor_
    // and stores them in the termios structure old_termios_.
    tcgetattr(file_descriptor_, &old_termios_);

    // copy the data from data_ready_ to raw_data_
    memcpy(&new_termios_, &old_termios_, sizeof(struct termios));
    // Set some local modes of termios structure to 0 (disable them):
    //  ~ECHO: disable Echo input characters (not showing them in terminal).
    //  ~ICANON : disable canonical mode (input is available immediately)
    new_termios_.c_lflag &= ~(ICANON | ECHO);

    // Define the terminal special characters
    //  VEOL: Additional end-of-line character
    //  VEOF: End-of-file character (causes the pending tty buffer to
    //        be sent to the waiting user program without waiting for
    //        end-of-line).
    new_termios_.c_cc[VEOL] = 1; // Start of heading (ctrl-A)
    new_termios_.c_cc[VEOF] = 2; // Start of text (ctrl-B)

    // sets the parameters associated with the terminal to new_termios_
    tcsetattr(file_descriptor_, TCSANOW, &new_termios_);

    ROS_INFO("Starting teleop_key node");
    ROS_INFO("%s", msg_);

    keyLoop();
}

void KeyInterpreter::keyLoop() {
    char c;
    while (1) {
        // attemps to read 1 character, exit on error
        if (read(file_descriptor_, &c, 1) < 0) {
            ROS_INFO("FAILED TO READ");
            perror("read():");
            exit(-1);
        }

        // TODO: remove switch case
        switch (c) {
            case KEY_UP:
                command_ = "FRONT";
                linear_factor_ = 1;
                angular_factor_ = 0;
                break;
            case KEY_DOWN:
                // TODO: move smoothly
                command_ = "BACK";
                linear_factor_ = -1;
                angular_factor_ = 0;
                break;
            case KEY_RIGHT:
                command_ = "TURN-RIGHT";
                linear_factor_ = 0;
                angular_factor_ = 1;
                break;
            case KEY_LEFT:
                command_ = "TURN-LEFT";
                linear_factor_ = 0;
                angular_factor_ = -1;
                break;
            case KEY_W:
                command_ = "Increased ls";
                linear_speed_ += 0.1;
                break;
            case KEY_S:
                command_ = "Decreased ls";
                linear_speed_ -= 0.1;
                if (linear_speed_ < 0) {
                    linear_speed_ = 0.0;
                }
                break;
            case KEY_D:
                command_ = "Increased as";
                angular_speed_ += 0.1;
                break;
            case KEY_A:
                command_ = "Decreased as";
                angular_speed_ -= 0.1;
                if (angular_speed_ < 0) {
                    angular_speed_ = 0.0;
                }
                break;
            case KEY_M:
                ROS_INFO("%s", msg_);
                break;
            default:
                ROS_INFO("value: 0x%02X not suported. Stopping.", c);
                command_ = "NS";
                angular_factor_ = linear_factor_ = 0;
                break;
        }

        publish();
    }
}

void KeyInterpreter::publish() {
    geometry_msgs::Twist vel_msg_;
    vel_msg_.linear.x = linear_factor_ * linear_speed_;
    vel_msg_.angular.z = angular_factor_ * angular_speed_;
    ROS_INFO("key: %s - ls: %f - as: %f", command_.c_str(), linear_speed_, angular_speed_);
    cmd_vel_publisher_.publish(vel_msg_);
}
