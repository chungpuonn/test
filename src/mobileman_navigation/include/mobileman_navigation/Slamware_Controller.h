/**
 * @file Slamware_Controller.h
 * @brief This file defines a class to control the behavior of slamware
 * @author Bingbing Li, bingbing.li@ntu.edu.sg
 * @version 0.0
 * @date 2020-10-30
 */


#ifndef SRC_SLAMWARE_CONTROLLER_H
#define SRC_SLAMWARE_CONTROLLER_H

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <slamware_ros_sdk/ClearMapRequest.h>
#include <slamware_ros_sdk/RecoverLocalizationRequest.h>
#include <slamware_ros_sdk/SetMapLocalizationRequest.h>
#include <slamware_ros_sdk/SetMapUpdateRequest.h>
#include <slamware_ros_sdk/SyncSetStcmRequest.h>
#include <slamware_ros_sdk/SyncSetStcm.h>

namespace mobileman {
using namespace std;
/**
 * @class Slamware_Controller
 * defines a ros controller for slamware
 * @example  example cosines = mobilman(radians(angles));
 */
class Slamware_Controller {
public:
/**
 * Explicit constructor of slamware_controller, need to provide ros nodehandler
 * @param nh nodehanle from ros
 */
  explicit Slamware_Controller(ros::NodeHandle &nh);
  virtual ~Slamware_Controller();

  /**
   * Send a command to clear current map.
   * @return 0 if succeed
   */
  int clearMap();
  /**
   * force the robot to locate itself during initialization.
   * @return 0 if succeed
   */
  int resetInitialLocation();
  /**
   * turn on the localization of slamware core.
   * @return 0 if succeed
   */
  int toggleLocalizationON();
  /**
   * turn on the localization of slamware core.
   * @return 0 if succeed
   */
  int toggleLocalizationOFF();
  /**
   * turn on the mapping of slamware core.
   * @return 0 if succeed
   */
  int toggleMappingON();
  /**
   * turn OFF the mapping of slamware core.
   * @return 0 if succeed
   */
  int toggleMappingOFF();
  /**
   * Load a map to slamware
   * @param mapPath the path of map file.
   * @return 0 if succeed and -1 if failed;
   */
  int uploadMap(std::string mapPath);

  /**
   * Initialize slamware, including turn off the mapping function, and load a map.
   * @return 0 if succeed
   */
  int slamwareInitialization();

private:
  ros::NodeHandle &nodeHandle;
  ros::Publisher clearMapPub =
      nodeHandle.advertise<slamware_ros_sdk::ClearMapRequest>(
          "/slamware_ros_sdk_server_node/clear_map", 1000, true);
  ros::Publisher toggleMappingPub =
      nodeHandle.advertise<slamware_ros_sdk::SetMapUpdateRequest>(
          "/slamware_ros_sdk_server_node/set_map_update", 1000, true);
  ros::Publisher toggleLocalizationPub =
      nodeHandle.advertise<slamware_ros_sdk::SetMapLocalizationRequest>(
          "/slamware_ros_sdk_server_node/set_map_localization", 1000, true);
  ros::Publisher forceLocalizationPub =
      nodeHandle.advertise<slamware_ros_sdk::RecoverLocalizationRequest>(
          "/slamware_ros_sdk_server_node/recover_localization", 1000, true);
  ros::ServiceClient mapUploaderSerClient =
      nodeHandle.serviceClient<slamware_ros_sdk::SyncSetStcm>(
          "/slamware_ros_sdk_server_node/sync_set_stcm");

  slamware_ros_sdk::ClearMapRequest clearMapMsg;
  slamware_ros_sdk::SetMapLocalizationRequest setMapLocalizationMsg;
  slamware_ros_sdk::SetMapUpdateRequest setMapUpdateMsg;
  slamware_ros_sdk::RecoverLocalizationRequest recoverLocalizationMsg;
  slamware_ros_sdk::SyncSetStcm mapUploaderService;
  std::string defaultMap =
      "/home/bingbingli/mobileman/src/mobileman_navigation/maps/sim.stcm";
};
}
#endif // SRC_SLAMWARE_CONTROLLER_H
