package ROS;


/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "Geometry_Twist" is a rosMessage type, Refer to <a href="http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html">Ros Documentation</a>.
 */

public class RosTopicMessage_MsgGeometry_Twist {


    private RosTopicMessage_MsgGeometry_Vector3 linear;
    private RosTopicMessage_MsgGeometry_Vector3 angular;

    public RosTopicMessage_MsgGeometry_Twist(RosTopicMessage_MsgGeometry_Vector3 linear, RosTopicMessage_MsgGeometry_Vector3 angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public RosTopicMessage_MsgGeometry_Vector3 getAngular() {
        return angular;
    }

    public void setAngular(RosTopicMessage_MsgGeometry_Vector3 angular) {
        this.angular = angular;
    }

    public RosTopicMessage_MsgGeometry_Vector3 getLinear() {
        return linear;
    }

    public void setLinear(RosTopicMessage_MsgGeometry_Vector3 linear) {
        this.linear = linear;
    }
}
