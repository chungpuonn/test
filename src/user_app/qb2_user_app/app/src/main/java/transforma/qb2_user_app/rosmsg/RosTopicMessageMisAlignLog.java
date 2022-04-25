package transforma.qb2_user_app.rosmsg;

import ROS.*;

/**
 * Gson object class: A class of a PicaBot specified rosMessage with its "msg" typed as uneven_misalignment, which is a special type of rsoMessage.
 * It contains two tyoe of defects information -unevenness and misalignment
 */


public class RosTopicMessageMisAlignLog extends RosTopicMessage {

    private MsgMisAlignLog msg;

    public RosTopicMessageMisAlignLog(String op, String topic, String type){
        super(op, topic, type);

    }

    public MsgMisAlignLog getMsg() {
        return msg;
    }

    /**
     * Gson object class: A class of the "msg" field of a RosTopicMessageMisAlignLog, and it has 7 attributes, "lippage_log" is a rosMessage type
     */

    public class MsgMisAlignLog{

        RosTopicMessage_MsgStd_Header header;
        int uneven_result;
        int misalignment_result;
        RosTopicMessage_MsgGeometry_Point uneven_loc;
        RosTopicMessage_MsgGeometry_Point misalignment_loc;
        float absolute_alignment_error;
        String image_filename;


        public RosTopicMessage_MsgStd_Header getHeader() {
            return header;
        }

        public int getUneven_result() {
            return uneven_result;
        }

        public int getMisalignment_result() {
            return misalignment_result;
        }

        public RosTopicMessage_MsgGeometry_Point getUneven_loc() {
            return uneven_loc;
        }

        public RosTopicMessage_MsgGeometry_Point getMisalignment_loc() {
            return misalignment_loc;
        }

        public float getAbsolute_alignment_error() {
            return absolute_alignment_error;
        }

        public String getImage_filename() {
            return image_filename;
        }
    }
}
