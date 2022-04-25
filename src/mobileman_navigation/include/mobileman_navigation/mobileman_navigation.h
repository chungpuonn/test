/**
 * @file mobileman_navigation.h
 * @author Bingbing LI (bingbing.li@ntu.edu.sg)
 * @brief This file defines the basic function for robot navigation
 * @version 0.1
 * @date 2021-03-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
namespace mobileman
{
    //TODO
    int wallFollowing();

    void modeSelection();

    int staircaseRecognition();

    int stairCounter();

    void laserScanCallback()
    {

        };

}
