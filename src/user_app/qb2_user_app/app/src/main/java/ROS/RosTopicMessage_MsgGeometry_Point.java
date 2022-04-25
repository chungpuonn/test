package ROS;

/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "Geometry_Point" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html">Ros Documentation</a>.
 */

public class RosTopicMessage_MsgGeometry_Point {

    //It should be using double instead of float, however, to reduced data size we use float here

    public float x;
    public float y;
    public float z;


}
