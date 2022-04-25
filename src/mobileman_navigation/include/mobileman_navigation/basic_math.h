/**
 * @file basic_math.h
 * @author bingbingLI (bingbing.li@ntu.edu.sg)
 * @brief file containing some commonly used math tools
 * @version 0.1
 * @date 2021-03-23
 *
 * @copyright Copyright (c) 2021
 *
 */


#ifndef INCLUDE_BASIC_MATH_H
#define INCLUDE_BASIC_MATH_H

#include <cstdlib>
#include <cstdio>
#include <vector>
#include <math.h>
#ifndef PI
#define PI 3.141593
#endif
namespace tools {
	/**
	 * @brief Simple math function for robotics related programming
	*/
	namespace math_tools {

		/**
		 * @brief a pair of integers 
		*/
		struct indexPair {
			/// begin of index
			int begin;	
			/// end of index
			int end;
		};
		/**
		 * @brief A polar coordinate structure
		*/
		struct polar {
			/// magnititute of the polar vector
			double distance; 
			/// angle of the polar vector
			double angle;	 
		};
		/**
		 * @brief a cartersian coordinate structure
		*/
		struct cartesian {
			/// x coordinate of the cartesian vector
			double x;
			/// y coordinate of the cartesian vector
			double y;	   
		};

		/**
		 * Return the average of the vaules held in a vector
		 * @param v vector holding the values to be calculatetd
		 * @return the average value
		 */
		double avg(std::vector<double>& v); 

		/**
		 * Calculate the variance of the values held in a vector
		 * @param v Vector holding the values to be calculate
		 * @param mean Population mean
		 * @return The variance
		 */
		double variance(std::vector<double>& v, double mean);
		/**
		 * @brief convert cartersian coordinate to polar coordinate
		 * @param coordinate polar coordinate to be converted
		 * @return corespoidng cartesian coordinate
		*/

		cartesian polar2Cartesian(polar coordinate);
		/**
		* @brief convert cartersian coordinate to polar coordinate
		* @param coordinate polar coordinate to be converted
		* @return corespoidng cartesian coordinate
	   */
		std::vector<cartesian> polar2Cartesian(const std::vector<polar>& coordinates);
		/**
		* @brief convert polar coordinate to cartersian coordinate
		* @param coordinate  cartesian coordinate to be converted
		* @return corespoidng polar coordinate
	   */
		polar cartesian2Polar(cartesian corrdinate);
		/**
		* @brief convert polar coordinate to cartersian coordinates
		* @param coordinate  cartesian coordinates to be converted
		* @return corespoidng polar coordinates
	   */
		std::vector<polar> cartesian2Polar(const std::vector<cartesian>& corrdinates);
	} //namespace math_tools
} // namespace tools
#endif // SRC_INCLUDE_MATH_H
