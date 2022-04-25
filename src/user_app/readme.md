# Qb2_User_App

This file contains the overview of the app design logic, file structure and other necessary points that following developers should be clear of.

## Overview

This app is designed and developed by an itern of the company and therefore the design logic and coding styles may need refinements and improvements. 


## Deployment  

Make sure u know basic ```git``` and ```Android Development``` Knowledge

1. Install Git and Android Studio on your laptop.(follow tutorial) 
2. Install Android SDK 29
3. ```git clone``` the repository to your local folder say ```src```
4. choose open an existing android studio project ```src/qb2_uesr_app/qb2_user_app``` as an ```Android Project```
5. Wait for all automatic initialisation to finish
6. Connects your Android Phones to pc with ```debugging mode``` as well as ```allow usb install```turned on in ```developer options``` setting page. However, this step is optional if you are using an emulator to build and test.
7. then u can run the app
8. To build the release mode apk for other phone use, refer to the key folder and scrrenshot to fill up apk key signature when relevant pop ups show

### Future development:  
1. More memory management required to load OpenGL view
2. Background service may be used to run TCP client so that the websocket connection may be more stable  
3. Multi threading tasking may be used
4. Login password encrption may be implemented
5. Find a way to enable mulit client connection with RosCore
6. Put a state management design pattern to entire projects

### Main Design Structure:

The app's main function is to send, receive and record data to/from the RosCore implemented on the Quicabot, by connecting our TCP java websocket client to the websocket server established by <a href="http://wiki.ros.org/rosbridge_server">RosBridgeServer</a> on the ROS. 

The app is able to command the robot to execute tasks, view robot status and collect different kind of infomrtaion such as point cloud, inspection results and mapping infos by subscribing to **respective rostopics** created on the RosCore. Topics subscription or pushlishing information to a certain topic is carried out by sending a formatted **json string** accepted by the RosBridgeServer.

Before entering a specific function, the app shoudl ensure it successfully subscribed to centain topics and otherwise error message should pop up. Before doing certain actions must ensure there will be success response and failure response and ensure that the robot is ready. Save resuable information to sharedpreference before quiting certain function. Make sure the java files have implemented rosbridgelisteners if they are to interect with the rosBridge.

**Make sure all UI files are configured to sensor landscape ond key in 'keepscreenon = true' parameter in the xml file to prevent screen off and rosbridge disconnects (can be prevented by implementing bg services, u can try implement it)**

## File Structure

📦java  
 ┣ 📂ROS  
 ┃ ┣ 📜RosBridgeConnection.java  
 ┃ ┣ 📜RosBridgeListener.java  
 ┃ ┣ 📜RosMsgRaw.java  
 ┃ ┣ 📜RosServiceMessage.java  
 ┃ ┗ 📜RosTopicMessage.java  
 ┗ 📂transforma  
 ┃ ┗ 📂qb2_user_app  
 ┃ ┃ ┣ 📂quicabotAppLicence  
 ┃ ┃ ┃ ┣ 📜AppLicenceUsed.java  
 ┃ ┃ ┃ ┣ 📜LicencePageRecyclerViewAdapter.java  
 ┃ ┃ ┃ ┗ 📜QuicabotAppLicencePage.java  
 ┃ ┃ ┣ 📂quicabotInspectionLog  
 ┃ ┃ ┃ ┣ 📜InspectionLog.java  
 ┃ ┃ ┃ ┣ 📜LogPageRecyclerviewAdapter.java  
 ┃ ┃ ┃ ┗ 📜QuicabotInspectionLogPage.java  
 ┃ ┃ ┣ 📂quicabotMap  
 ┃ ┃ ┃ ┣ 📜MapData.java  
 ┃ ┃ ┃ ┣ 📜MapRenderer.java    
 ┃ ┃ ┃ ┣ 📜MapView.java  
 ┃ ┃ ┃ ┗ 📜QuicabotMapPage.java  
 ┃ ┃ ┣ 📂quicabotPointcloud  
 ┃ ┃ ┃ ┣ 📜PointCloudRenderer.java  
 ┃ ┃ ┃ ┣ 📜PointCloudView.java  
 ┃ ┃ ┃ ┗ 📜QuicabotPointCloudPage.java  
 ┃ ┃ ┣ 📂rosmsg  
 ┃ ┃ ┃ ┣ 📜InspectionResult.java  
 ┃ ┃ ┃ ┣ 📜InspectionResultFormal.java  
 ┃ ┃ ┃ ┣ 📜RosServiceMessageAddress.java  
 ┃ ┃ ┃ ┣ 📜RosServiceMessageGenReport.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessageCmdVel.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessageCrackResult.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessageLippageLog.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessageMap.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessageMisAlignLog.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessagePanTiltCommand.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessagePanTiltState.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessagePointCloud.java  
 ┃ ┃ ┃ ┣ 📜RosTopicMessagePoseUpdate.java  
 ┃ ┃ ┃ ┗ 📜RosTopicMessageString.java  
 ┃ ┃ ┣ 📜ConfigurationParameters.java  
 ┃ ┃ ┣ 📜QuicabotAbout.java  
 ┃ ┃ ┣ 📜QuicabotAppCompatActivity.java  
 ┃ ┃ ┣ 📜QuicabotControllerMode.java  
 ┃ ┃ ┣ 📜QuicabotGenerateReportSetting.java  
 ┃ ┃ ┣ 📜QuicabotInspectionMode.java  
 ┃ ┃ ┣ 📜QuicabotInspectionResult.java  
 ┃ ┃ ┣ 📜QuicabotLogin.java  
 ┃ ┃ ┣ 📜QuicabotMainPage.java  
 ┃ ┃ ┣ 📜Utils.java  
 ┃ ┃ ┗ 📜UtilsOpenGl.java  


 The above tree shows the entire structure of our app. It has two main modules:  

 1. ROS  
   
     The ```ROS``` module is an independent module that is responsible for establishing rosBridge client, serialise and desirialise json string used for data transfer. It also cantains many classes of common used RosTopics (official rostopics only) **json object class** for easier serialisation and deserialisation.  
     In this module, ```RosBridgeConnection``` class is an **singleton** class used as websocket client and **observer** and ```RosBridgeListener``` is an **listener** interface that all **app function java files** would implement so as to get notified by any data received. 


 2. transforma/qb2_user_app  


     The ```transforma``` module is an module of all app function files and all other **Quicabot only** rosTopic json object files.

     1. ```quicabotAppLicence```, ```quicabotInspectionLog``` are module that incharge of displaying app licence page and inspection Log page, the log page need a lot **TODO**
     2. ```quicabotMap```, ```quicabotPointcloud``` are two submudules that uses OpneGl ES to display 3d graphics. The mapping mudlue shows a top down view of a 2d mapping generated by the robot. And pointcloud submodule is used to display the 3d scanning result of the robot. All raw information is received from the rosBridge and desirialised to relative point and color information into buffers for OpenGl ES display.
     3. ```ROSMSG``` is a submodule of all Quicabot defined RosTopics for inspection results. All Quicatbot of Pictobot only topics should not go into ```ROS``` module.  
   
     4. Other files in the mother modules such as ```QuicabotMainPage.java``` are UI java files for the app. ```Util.java```, ```UtilsOpenGl.java```, are utils:) !
     5. Inside ```ConfigurationParameters.java``` u can adjust some of the consts for the app and other constants such as **topic subscription name and topic types** rest inside ```RosBridgeListener.java```, you can adjust it there.

2. Androidmanifext.xml
   
   This file contains all activity and permissino declaration information, when creating a new activity u need to configure the **sensor landscape** declaration here. If u need any other permission here, remember to declar it here

3. build.gradle(Module:APP)
   
   This file contains compile information, such as ur build ```SDK version, app version, dependecies``` and so on.

4.
