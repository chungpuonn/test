/**
*	@file tools.h
*	@author bingbingLI (bingbing.li@ntu.edu.sg)
*   @brief This file defines basic tools for ros to the robot
*   @version 0.1
*   #data 2021-3-24
*/

#ifndef TOOLS_H
#define TOOLS_H

#include <cstdio>
#include <iostream>
#include <sstream>
#include <iomanip>
#ifdef __linux__
#include <unistd.h>
#define GetCurrentDir getcwd
#else
#include <dirent.h>
#define GetCurrentDir _getcwd
#endif 

#include <boost/range/irange.hpp>

#include <ros/ros.h>

#include "basic_math.h"
/**
 * @brief a namespace tools contains basic tools for robotic programing, currently includes
 * two parts ros related tools and math related tools
*/
namespace tools
{
	/**
	 * @brief ros related tools for robotics relate programming
	*/
	namespace ros_tools 
	{
		/**
		 * @brief initialze a topic, wait for subscriber and proceed when its ready
		 * @param pub ros publisher name
		 * @param funcName function name
		*/
		void initTopic(ros::Publisher pub, std::string funcName);
	}

        /**
        * @brief get the indexes for the laser scan points, containing indicated clips for eight direction
        * @param range the angular range each clip has
        * @param increment increment of laser scan, given by laser scanner
        * @param numberOfClips  number of clips to be used, default is 8
        * @return index pairs for the clip
        */
        std::vector <math_tools::indexPair> getIndex(double range, float increment, int numberOfClips = 8);

	/**
	 * @brief Get the current working direcotry as string
	 * @param
	 * @return Current working directory of the file in std::string format
	*/
	std::string getWorkingDirecotry(void);

	/**
	 * @brief  Get the current Date and Time as string
	 * @param
	 * @return Current Date and Time in std::string format
	*/
	std::string getCurrentDateTime(void);
}

#endif // TOOLS_H

