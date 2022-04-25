/**
 * @file wall_following.h
 * @author bingbingLI (bingbing.li@ntu.edu.sg)
 * @brief This file defines a class called wall_following for the methods and  elements used in wallfollowing algorithm
 * @version 0.1
 * @date 2021-02-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
//
// Created by bingbingli on 5/2/21.
//

#ifndef SRC_WALL_FOLLOWING_H
#define SRC_WALL_FOLLOWING_H

#include <cstdlib>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>

#include "mobilerobot.h"
#include "tools.h"

namespace mobileman
{
  class Wall_Following: public MOBILE_ROBOT
  {

  public:
    /**
     * Default topic name for the velocity command to be published
     */
    std::string default_vel_cmd = "/mobileman_navigation/vel/sonar";
    virtual ~Wall_Following();

    Wall_Following(ros::NodeHandle);
    /**
     * @param nh Nodehanle from ros
     * @param topicname velocity topic to publish to
     */
    Wall_Following(ros::NodeHandle, std::string );

    /**
     * Constructor of wall_following class with embedded subcriber and publisher
     * @param n Nodehandle passed from ros node
     * @param subscribeTopic subscribe topic to get sonar data
     * @param publishTopic  pushlish topic to pushlish velocity command
     */
    Wall_Following(ros::NodeHandle n, const std::string& subscribeTopic, std::string publishTopic);

    /**
     * Calculate the desire velocity based on sonar reading.
     * @param sonarReadings
     * @return desired velocity of robot to be used.
     */
    static geometry_msgs::Twist calculateVelocity(std::vector<float> sonarReadings);
    /**
     * Return the two sonar reading to be used for calculate the head angle of the robot
     * @return R2 float vector contains the readings of the two sonar angle
     */
    static std::vector<float> selectSonarData(std::vector<float>);

    /**
     * Calculate current heading with regarding to all
     * @param sonar1 reading of sonar 1 in meter
     * @param sonar2 reading of sonar 2 in meter
     * @return angle towards the wall, positive means the robot moves towards the all
     */
    static double calculateRobotHeading(double sonar1, double sonar2);

    /**
     * Callback funtion to generate robot speed using Sonar, the velocity command will be published to
     * a topic, default at "/mobileman_navigation/vel/sonar"
     * @param msg the ros message of sonar datas
     * @return null
     */
     void wallFollowingUsingSonar(const std_msgs::Float32MultiArray::ConstPtr& msg);

     /**
      * Callback function to generate  robot speed using laser, the velocity command will be published to
      * defined topic "/mobileman_navigation/vel/sonar"
      * @param scan laser scan obtained from laser scanner
      */
     void wallFollowingUsingLaser(const sensor_msgs::LaserScan scan);
     
     /**
      * @brief select laser scan clips from an entire laser scan for information in different direction of the robot
      * @param scan laser scan to be extracted from
      * @return laser scan clips
     */
     std::vector<float> selectLaserData(sensor_msgs::LaserScan scan);
     /**
     * Publish the velocity to default_vel_cmd
     * @param velocity
     */
     void publishVelocity(geometry_msgs::Twist velocity);
  private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;

  };
}
#endif // SRC_WALL_FOLLOWING_H
