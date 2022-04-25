package transforma.qb2_user_app.rosmsg;

import ROS.*;
/**
 * Gson object class: A class of a PicaBot specified rosMessage with its "msg" typed as crackResult, which is a special type of rsoMessage
 */


public class RosTopicMessageCrackResult extends RosTopicMessage {

    MsgCrackResult msg;

    public RosTopicMessageCrackResult(String op, String topic, String type) {
        super(op, topic, type);
    }

    public MsgCrackResult getMsg() {
        return msg;
    }

    /**
     * Gson object class: A class of the "msg" field of a RosTopicMessageCrackResult, and it has 4 attributes, "crack_result" is a rosMessage type
     */

    public class MsgCrackResult {

        int crack_result;
        RosTopicMessage_MsgStd_Header header;
        RosTopicMessage_MsgGeometry_Point crack_loc;
        String image_filename;

        public int getCrack_result() {
            return crack_result;
        }

        public RosTopicMessage_MsgStd_Header getHeader() {
            return header;
        }

        public RosTopicMessage_MsgGeometry_Point getCrack_loc() {
            return crack_loc;
        }

        public String getImage_filename() {
            return image_filename;
        }
    }
}
