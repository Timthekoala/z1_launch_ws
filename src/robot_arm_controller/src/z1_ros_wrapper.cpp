#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "unitree_arm_sdk/control/unitreeArm.h"

#include <termios.h>
#include <unistd.h>
#include <iostream>

#include <thread>
#include <atomic>

char getch();

UNITREE_ARM::unitreeArm* armPtr = nullptr; // Pointer to the arm object
double gripper = 0.0;
double gripper_vel = 0.0;
bool pause_arm = false;

ros::Publisher gripperTauPub; // Declare a publisher

std::atomic<bool> exit_flag(false); // Flag to signal exit

void keyboardInputThread() {
    while (!exit_flag) {
        char c = getch();
        if (c == 'h') {
            pause_arm = true;
            armPtr->labelRun("startFlat");
            armPtr->startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);
            ROS_INFO("Going to home");
            pause_arm = false;
        }
        if (c == 'f') {
            pause_arm = true;
            armPtr->labelRun("forward");
            armPtr->startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);
            ROS_INFO("Going to forward position");
            pause_arm = false;
        }
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Example usage: setting some arm commands based on the incoming Twist message
    // Here we simply print the received velocity commands

    if (armPtr)
    {
        // Mapping the velocity command to Cartesian control directions
        Vec7 directions;
        /*directions << msg->angular.x, msg->angular.y, msg->angular.z, 
                      msg->linear.x, msg->linear.y, msg->linear.z, gripper;*/ // Assuming 0.0 for the 7th component
        directions << msg->angular.x, msg->angular.y, msg->angular.z, 
                      msg->linear.x, msg->linear.y, msg->linear.z, 0.0;
        
        /* ROS_INFO("Received cmd_vel: linear x: [%f], y: [%f], z: [%f]; angular x: [%f], y: [%f], z: [%f], Gripper:[%f]",
        msg->linear.x, msg->linear.y, msg->linear.z,
        msg->angular.x, msg->angular.y, msg->angular.z, gripper);
          */

        double angular_vel = 1.2; // Example orientation speed
        double linear_vel = 1.2; // Example position speed

        //armPtr->startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);
        if (!pause_arm){
            armPtr->cartesianCtrlCmd(directions, angular_vel, linear_vel);
            armPtr->setGripperCmd(gripper, gripper_vel, 0);
        }
        
    }
}

// Function to publish gripper torque
void publishGripperTau() {
    if (armPtr) {
        double gripper_tau = armPtr->lowstate->getGripperTau();
        std_msgs::Float32 tauMsg;
        tauMsg.data = gripper_tau;
        gripperTauPub.publish(tauMsg);
        ROS_INFO("Published Gripper Tau: %f", gripper_tau);
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

        if (msg->buttons[0] == 1 && msg->buttons[1] == 1 && gripper < 0.0 ) // Both buttons pressed to close gripper
        {
            gripper += 0.01;
            gripper_vel = 0.1;
        }
        else if (((msg->buttons[0] == 1 && msg->buttons[1] == 0) ||  (msg->buttons[0] == 0 && msg->buttons[1] == 1)) && gripper > -1.0) // 1 button press to open gripper
        {
            gripper += -0.01;
            gripper_vel = 0.1;
        }
        else
        {
            gripper_vel = 0.0;
        }
}

// Function to handle keyboard input
char getch()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "unitree_arm_controller");
    ros::NodeHandle nh;

    ROS_INFO("Starting unitree_arm_controller node");

    // Initialize the arm
    UNITREE_ARM::unitreeArm arm(true);
    armPtr = &arm; // Assign the pointer to the arm object
    arm.sendRecvThread->start();
    // arm.backToStart();
    arm.startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);


    // Create a publisher for the haptic palm topic
    gripperTauPub = nh.advertise<std_msgs::Float32>("/haptic/palm", 10);

    // Subscribe to the /cmd_vel topic
    ros::Subscriber sub = nh.subscribe("/spacenav/twist", 10, cmdVelCallback);

    ROS_INFO("Subscribed to /spacenav/twist");

    ros::Subscriber subGripper = nh.subscribe("/spacenav/joy", 10, joyCallback);

    ROS_INFO("Subscribed to /spacenav/joy");

    // ROS spin to keep the callback function alive
    ros::AsyncSpinner spinner(1); // Use a spinner to handle ROS callbacks
    spinner.start();

    std::thread keyboard_thread(keyboardInputThread);

    ros::Rate rate(10);
    while (ros::ok()) {
        publishGripperTau();
        rate.sleep();
    }

    //Clean up
    //arm.backToStart();
    //arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    exit_flag = true; // Signal the keyboard thread to exit
    keyboard_thread.join(); // Wait for the keyboard thread to finish
    arm.sendRecvThread->shutdown();

    ROS_INFO("Shutting down unitree_arm_controller node");

    return 0;
}

