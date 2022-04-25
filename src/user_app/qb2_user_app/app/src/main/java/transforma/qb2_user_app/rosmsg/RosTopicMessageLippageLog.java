package transforma.qb2_user_app.rosmsg;

import ROS.*;

/**
 * Gson object class: A class of a PicaBot specified rosMessage with its "msg" typed as lippage_log, which is a special type of rsoMessage
 */

public class RosTopicMessageLippageLog extends RosTopicMessage {

    RosTopicMessage_MsgLippageLog msg;

    public RosTopicMessageLippageLog(String op, String topic, String type){
        super(op,topic,type);
    }

    public RosTopicMessage_MsgLippageLog getMsg() {
        return msg;
    }

    /**
     * Gson object class: A class of the "msg" field of a RosTopicMessageLippageLog, and it has 4 attributes, "lippage_log" is a rosMessage type
     */

    public class RosTopicMessage_MsgLippageLog {

        int lippage_result;
        RosTopicMessage_MsgStd_Header header;
        RosTopicMessage_MsgGeometry_Point lippage_loc;
        String image_filename;

        public RosTopicMessage_MsgGeometry_Point getLippage_loc() {
            return lippage_loc;
        }
        public int getLippage_result() { return lippage_result; }
    }
}
