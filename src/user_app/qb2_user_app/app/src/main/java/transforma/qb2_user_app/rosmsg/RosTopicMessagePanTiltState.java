package transforma.qb2_user_app.rosmsg;

import ROS.RosTopicMessage;
import ROS.RosTopicMessage_MsgSensorMsgs_JointState;


/**
 * Gson object class: A class of a PicaBot specified rosMessage with its "msg" typed as SensorMsgs_jointState points, which is a special type of rsoMessage.
 * This class is used to contain information for pan-tilt system position information
 */

public class RosTopicMessagePanTiltState extends RosTopicMessage {

    public RosTopicMessage_MsgSensorMsgs_JointState getMsg() {
        return msg;
    }

    public void setMsg(RosTopicMessage_MsgSensorMsgs_JointState msg) {
        this.msg = msg;
    }

    private RosTopicMessage_MsgSensorMsgs_JointState msg;

    public RosTopicMessagePanTiltState(String op, String topic, String type) {
        super(op, topic, type);
    }


}
