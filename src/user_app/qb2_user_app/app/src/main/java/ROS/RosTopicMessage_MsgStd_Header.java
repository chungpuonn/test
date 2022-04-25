package ROS;

/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "SensorStd_Header" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html">Ros Documentation</a>.
 */

public class RosTopicMessage_MsgStd_Header {

    private String frame_id;
    //Note that the seq is in uint32, a long is required when in actual use
    private int seq;
    private MsgStamp stamp;


    public String getFrame_id() {
        return frame_id;
    }

    public int getSeq() {
        return seq;
    }

    public MsgStamp getStamp() {
        return stamp;
    }

    /**
     * Gson object class: A inner class of the "msg" field of a rosTopicMessage, just as an std_msgs/String, "stamp" is a rosMessage type
     */

    private class MsgStamp{
        public int secs;
        public int nsecs;
    }

}
