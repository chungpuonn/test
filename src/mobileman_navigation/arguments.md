##Usage
1. use mobileman_navigation.launch for the navigation
2. use mobielman_simulation simulation.launch to launch a simulated enviromen, or 
3. use bringup.launch to use the real robot

### arguments for navigation packages

1. withViewer<br>
   whether to enable rviz
   i. ture
   ii. false

2. mode<br>
   the mode of running lunch file
   1. mapping
   2. localization
    3. navigation 

3. useDummyLink<br>
   whether to use a dummy link for the interia of the baselink
    1. true
    2. false
 
4. useEKF <br>
    Whether to use robotekf
    1. true
    2. false
 
5. useLaserScan <br>
    Whether to use laserScan
    1. true
    2. false
    
6. robotName <br>
    Name of the robot, default is "mobileman"
   
7. useSimulation <br>
    whether to use simulation enviroment
   1. true
    2. false
    

   