/**
 * @file TF_BROADCASTER.h
 * @brief Defines the class used for declaration and broadcasting of TF transforms
 * @author Bingbing Li, bingbing.li@ntu.edu.sg
 * @version 0.1
 * @date 2020-10-21
 */

#ifndef SRC_TF_BROADCASTER_H
#define SRC_TF_BROADCASTER_H

#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
/*!
 * @class Tf_Broadcaster
 * @brief combines the declaration and broadcast of fixed tf transform
 * @defgroup MobileMan
 * @title MobileMan
 */
class Tf_Broadcaster
        {
        public:
        Tf_Broadcaster(std::string parent_link, std::string child_link, tf::TransformBroadcaster &br);
        ~Tf_Broadcaster();
        std::string parent_link;
        std::string child_link;
        tf::Transform transform;
        int sendTransform();

        private:
        tf::TransformBroadcaster &br;
        };


#endif //SRC_TF_BROADCASTER_H
