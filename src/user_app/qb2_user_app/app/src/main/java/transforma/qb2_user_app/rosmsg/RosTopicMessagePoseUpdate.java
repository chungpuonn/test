package transforma.qb2_user_app.rosmsg;

import ROS.RosTopicMessage;
import ROS.RosTopicMessage_MsgGeometry_PoseWithConvarianceStamped;

public class RosTopicMessagePoseUpdate extends RosTopicMessage {

    private RosTopicMessage_MsgGeometry_PoseWithConvarianceStamped msg;

    public RosTopicMessagePoseUpdate(String op, String topic, String type) {
        super(op, topic, type);
    }

    public RosTopicMessage_MsgGeometry_PoseWithConvarianceStamped getMsg() {
        return msg;
    }

    public void setMsg(RosTopicMessage_MsgGeometry_PoseWithConvarianceStamped msg) {
        this.msg = msg;
    }
}
