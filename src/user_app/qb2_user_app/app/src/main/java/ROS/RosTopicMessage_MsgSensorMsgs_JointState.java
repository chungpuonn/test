package ROS;


/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "SensorMsgs_JointState" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html">Ros Documentation</a>.
 */


public class RosTopicMessage_MsgSensorMsgs_JointState {


    private RosTopicMessage_MsgStd_Header header;
    private String[] name;
    private double[] position, velocity, effort;

    public RosTopicMessage_MsgStd_Header getHeader() {
        return header;
    }

    public void setHeader(RosTopicMessage_MsgStd_Header header) {
        this.header = header;
    }

    public String[] getName() {
        return name;
    }

    public void setName(String[] name) {
        this.name = name;
    }

    public double[] getPosition() {
        return position;
    }

    public void setPosition(double[] position) {
        this.position = position;
    }

    public double[] getVelocity() {
        return velocity;
    }

    public void setVelocity(double[] velocity) {
        this.velocity = velocity;
    }

    public double[] getEffort() {
        return effort;
    }

    public void setEffort(double[] effort) {
        this.effort = effort;
    }
}
