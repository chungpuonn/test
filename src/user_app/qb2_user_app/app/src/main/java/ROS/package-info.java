/**
 * <p>
 *     ROS is an rosBridge client implementation for android applications. With Ros, you can communicate with rosBridge through rosTopic Messages and rosServices
 *
 * </p>
 * <p>
 *     It is strongly suggested to read <a href="https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md">RosBridge Protocol</a> before reading the doc and code
 * </p>
 *  <p>
 *      It has two Parts, first part is a RosBridge Listener interface and a messaging handling class, which deal with the messaging
 *      The second part are all rosMessage class: A class named as RosTopicMessage_MsgTYPE means it is a complete string field for msg after get serialised
 *  </p>
 *
 *
 *  @author Transforma Robotics  - Lyu Zizheng
 *  @use org.java-websocket:Java-WebSocket:1.4.0
 *  @use com.google.code.gson:gson:2.8.5

 */
package ROS;