package ROS;

public class RosTopicMessage_MsgGeometry_PoseWithConvarianceStamped {


    RosTopicMessage_MsgStd_Header header;
    RosTopicMessage_MsgGeometry_PoseWithCovariance pose;


    public RosTopicMessage_MsgStd_Header getHeader() {
        return header;
    }

    public void setHeader(RosTopicMessage_MsgStd_Header header) {
        this.header = header;
    }

    public RosTopicMessage_MsgGeometry_PoseWithCovariance getPose() {
        return pose;
    }

    public void setPose(RosTopicMessage_MsgGeometry_PoseWithCovariance pose) {
        this.pose = pose;
    }
}
