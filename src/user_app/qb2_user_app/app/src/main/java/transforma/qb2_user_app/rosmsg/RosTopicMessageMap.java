package transforma.qb2_user_app.rosmsg;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import ROS.RosTopicMessage;
import ROS.RosTopicMessage_MsgNavMsgs_OccupancyGrid;

public class RosTopicMessageMap extends RosTopicMessage {


    private RosTopicMessage_MsgNavMsgs_OccupancyGrid msg;



    public RosTopicMessageMap(String op, String topic, String type) {
        super(op, topic, type);
    }


    public RosTopicMessage_MsgNavMsgs_OccupancyGrid getMsg() {
        return msg;
    }

    public void setMsg(RosTopicMessage_MsgNavMsgs_OccupancyGrid msg) {
        this.msg = msg;
    }
}
