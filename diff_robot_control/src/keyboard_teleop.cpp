/**
 * Define node
 * that detects keys being pressed
 * and sends messages accordingly
 */

#include <signal.h>
#include <memory>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <KeyInterpreter/KeyInterpreter.hpp>

// Global pointer to an interpreter, more info below.
std::unique_ptr<KeyInterpreter> interpreter_ptr;

// Declaration of quit. Needed for stopping the interpreter
// in a custom way
void quit(int sig);

int main(int argc, char **argv) {
    // Initiate the node
    ros::init(argc, argv, "keyboard_teleop");

    /**
     * Instantiate the class: this is a bit tricky, I do it like this
     * cause I need a global pointer to use signal to stop the internal
     * getchar process in the class. See https://stackoverflow.com/a/50863831
     */
    interpreter_ptr = std::make_unique<KeyInterpreter>();

    /**
     * Here, the normal thing is to do while(ros::ok()), however, we want
     * to stop the buffer receiving char so I will catch the system
     * signal to execute a custom shutdown.
     */
    signal(SIGINT, quit);

    /**
     * run the loop in which the interpreter parses the key and execute
     * the corresponding action.
     */
    interpreter_ptr->interpreteKeys();

    ROS_INFO("Good bye !");

    return 0;
}

void quit(int sig) {
    (void)sig;               // just using sig to don't have compilation errors
    interpreter_ptr->quit(); // calling the real quit
}
