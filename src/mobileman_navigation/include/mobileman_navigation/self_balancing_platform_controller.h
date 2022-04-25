/**
 * @file self_balancing_platform_controller.h
 * @author bingbingLI (bingbing.li@ntu.edu.sg)
 * @brief Defining the self balancing platform controller class
 * @version 0.1
 * @date 2021-0-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SRC_SELF_BALANCING_PLATFORM_CONTROLLER_H
#define SRC_SELF_BALANCING_PLATFORM_CONTROLLER_H
#define SPF_TOGGLE_TOPIC "/hold_SBP"
#define SPF_ANGLE_TOPIC  "/angles"

#include <iostream>

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

namespace mobileman {
class Self_Balancing_Platform_Controller {

public:
  /**
   * constructor of the class, ros nodehandle must be passed to construct the class
   * @param nh ROS node hanlder
   */
  explicit Self_Balancing_Platform_Controller(ros::NodeHandle &nh);
  virtual ~Self_Balancing_Platform_Controller();
/**
 * Toggle the status of Self balancing platform
 * @param hold ture to hold the platform, false to move the platform
 * @return
 */
  int toggleSPFHold(bool hold);
  /**
   * Control the self balancing platform each motors individually by emit angles. Note: abs(leftAngle - rightAngle)<=20 deg
   * @param leftAngle  Range from 190 to 100 deg
   * @param rightAngle Range from 190 to 100 deg
   * @return
   */
  int sendAngle(int leftAngle, int rightAngle);

private:
  ros::NodeHandle &nodeHandle;
  ros::Publisher SPFTogglePub = nodeHandle.advertise<std_msgs::Int16>(
      SPF_TOGGLE_TOPIC, 1,false
      );
  ros::Publisher SPFAnglePub = nodeHandle.advertise<std_msgs::Int32>(
      SPF_ANGLE_TOPIC, 1,false
      );

  std_msgs::Int16 SPFStatusMsg;
  std_msgs::Int32 SPFAngleMsg;
};
}
#endif // SRC_SELF_BALANCING_PLATFORM_CONTROLLER_H
