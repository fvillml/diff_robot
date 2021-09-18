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
    // sets the parameters back to old_termios_
    tcsetattr(file_descriptor_, TCSANOW, &old_termios_);
    ROS_INFO("Going to stop reading...");
    ros::shutdown();
    exit(0);
}

void KeyInterpreter::interpreteKeys() {
    /** gets the parameters associated with the object referred by file_descriptor_
     * and stores them in the termios structure old_termios_.
     */
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

        switch (c) {
            case KEY_UP:
                ROS_INFO("UP -> ls: %f, as: %f", linear_speed_, angular_speed_);
                lin_mult_ = 1;
                break;
            case KEY_DOWN:
                ROS_INFO("DOWN -> ls: %f, as: %f", linear_speed_, angular_speed_);
                lin_mult_ = -1;
                break;
            case KEY_RIGHT:
                ROS_INFO("RIGHT -> ls: %f, as: %f", linear_speed_, angular_speed_);
                ang_mult_ = 1;
                break;
            case KEY_LEFT:
                ROS_INFO("LEFT -> ls: %f, as: %f", linear_speed_, angular_speed_);
                ang_mult_ = -1;
                break;
            case KEY_W:
                linear_speed_ += 0.5;
                ROS_INFO("Increased linear speed to %f", linear_speed_);
                break;
            case KEY_S:
                linear_speed_ -= 0.5;
                if (linear_speed_ < 0)
                    linear_speed_ = 0.0;
                ROS_INFO("Decreased linear speed to %f", linear_speed_);
                break;
            case KEY_D:
                angular_speed_ += 0.5;
                ROS_INFO("Increased angular speed to %f", angular_speed_);
                break;
            case KEY_A:
                angular_speed_ -= 0.5;
                if (linear_speed_ < 0)
                    linear_speed_ = 0.0;
                ROS_INFO("Increased angular speed to %f", angular_speed_);
                break;
            case KEY_M:
                ROS_INFO("%s", msg_);
                break;
            default:
                ROS_INFO("value: 0x%02X not suported. Stopping.", c);
                ang_mult_ = lin_mult_ = 0;
                break;
        }

        publish();
    }
}

void KeyInterpreter::publish() {
    geometry_msgs::Twist vel_msg_;
    vel_msg_.linear.x = lin_mult_ * linear_speed_;
    vel_msg_.angular.z = ang_mult_ * angular_speed_;

    cmd_vel_publisher_.publish(vel_msg_);
}
