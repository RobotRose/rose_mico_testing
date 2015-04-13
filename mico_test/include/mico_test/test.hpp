/*!
 * \jaco_arm_trajectory_node.h
 * \brief Provides for trajectory execution and gripper control of the JACO arm.
 *
 * jaco_arm_trajectory_node creates a ROS node that provides trajectory execution and gripper 
 * control through the Kinova API, and smooth trajectory following through a velocity controller.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Mitchell Wills, WPI - mwills@wpi.edu
 */

#ifndef MICO_TEST_H_
#define MICO_TEST_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <boost/foreach.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <ecl/geometry.hpp>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <time.h>

#include <jaco_sdk/Kinova.API.UsbCommandLayerUbuntu.h>

#define SHOW_INFO_TIME             30.0 //[Hz]
#define SWITCH_ANGULAR_MODE_TIME   0.33 //[Hz]
#define SWITCH_CARTESIAN_MODE_TIME 15.0 //[Hz]
#define MAX_WAIT_TIME              10.0 //[Hz]

//gains for finger controller
#define KP_F 7.5
#define KV_F 0.05
#define KI_F 0.1

namespace mico_test {

class MicoTest
{
  public:
    MicoTest(std::string name, ros::NodeHandle n);
    virtual ~MicoTest();

  private:
    void setCartesianMode();
    void setAngularMode();
    void showInfo();

    void fingerPositionControl(float f1, float f2, float f3);
    void armPositionControl();

    ros::NodeHandle n_;
    std::string     name_;

    ros::Timer send_angular_mode_timer_; 
    ros::Timer send_cartesian_mode_timer_; 
    ros::Timer show_info_timer_; 

    boost::recursive_mutex api_mutex;

    int current_control_mode_;
};

} // namespace

#endif
