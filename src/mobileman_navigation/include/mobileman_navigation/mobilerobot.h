/**
 * @file mobilerobot.h
 * @author bingbingLI (bingbing.li@ntu.edu.sg)
 * @brief defining the mobilerobot class, with common properties and functions for an mobile robot
 * @version 0.1
 * @date 2021-03-1
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SRC_MOBILEROBOT_H
#define SRC_MOBILEROBOT_H

#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "system_log.hpp"
#include "tools.h"
#include "TF_BROADCASTER.h"

class MOBILE_ROBOT {
public:

  /**
   * @brief An object to log the event to a file
  */
  mobileman::Logger log;

  /**
   * @brief A pointer array holds all the static tf_broadcasters
  */
  std::vector <std::unique_ptr<Tf_Broadcaster>> tf_broacasters;
  /**
   * @brief current lasesr scan from the robot's laser scanner 
  */
  sensor_msgs::LaserScan current_scan;
  /**
   * length of the robot
   */
  static float length;
  /**
   * width of the robot
   */
  static float width;
  /**
   * Maximum linear speed of the robot
   */
  constexpr const static float max_linear_speed = 0;
/**
 * Maximum angular speed of the robot
 */
  constexpr const static float max_angular_speed = 0;
  /**
   * distance between sonar sensors on each side
   */
  static constexpr double sensor_distance = 0.25; // in meter

  //sensor_msgs::
  
  /**
   * Get battery level of the robot
   * @return 0-5, with 5 as full
   */
  int getBatteryLevel();

  /**
   * Get serial number of the robt
   * @return serial number as a string
   */
  std::string getSerialNumber();

  /**
   * Regulate input velocity so it does not exceed the maximum speed or less than the mimimum speed
   * @return Regulated velocity
   * example regulated_vel = velocityRegulation(orginal_vel)
   */
  static geometry_msgs::Twist velocityRegulation(geometry_msgs::Twist);

  /**
   * Default constructor
   * @example MOBILE_ROBOT my_robot;
   */
  MOBILE_ROBOT();
  /**
   * Constructor of mobile_robot pasing a string
   * @param 
   */
  MOBILE_ROBOT(const ros::NodeHandle &nh, std::string serial_number = "prototype2");
  ~MOBILE_ROBOT();

private:
  
  int battery_level;
  
  // Ros related
  ros::NodeHandle _nh;
  ros::Subscriber laser_sub;

  std::string _serial_number{};
  std::string sonarTopic;
  float sonarData[6];
  void getSonarDataCallBack(ros::NodeHandle & nh);
  /**
   * @brief initilize the laser topic to receive laser data
   * @param laserTopic
   * @return 0 if successed
  */
  int initializeLaser(std::string laserTopic = "/laser");
  void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

};

#endif // SRC_MOBILEROBOT_H
