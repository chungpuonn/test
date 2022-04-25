package ROS;


/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "Geometry_Vector3" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html">Ros Documentation</a>.
 */

public class RosTopicMessage_MsgGeometry_Vector3 {


    private double x;
    private double y;
    private double z;

    public RosTopicMessage_MsgGeometry_Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}
