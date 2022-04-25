/**
 * @file mobileman.h
 * @author bingbingLI (bingbing.li@ntu.edu.sg)
 * @brief This file defines the mobileman namespace with some basic properties of the robot
 * @version 0.1
 * @date 2021-03-23 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SRC_MOBILEMAN_H
#define SRC_MOBILEMAN_H

#include <ros/ros.h>

#include "system_log.hpp"
#include "mobilerobot.h"
/**
 * @brief namespace of the project. Mobileman, as a staircase climbing robot
*/
namespace mobileman
{
// TOPIC DEFINATION
#define SONAR_TOPIC "/sonar_data"
	/**
	 * @brief Class MOBILEMAN defines the basic class of this project
	*/
	class MOBILEMAN : public MOBILE_ROBOT
	{
	public:

		MOBILEMAN(const ros::NodeHandle& nh) ;

		~MOBILEMAN();

	private:
		ros::NodeHandle _nh;
		
	};
}


#endif // SRC_MOBILEMAN_H
