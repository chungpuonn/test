package transforma.qb2_user_app.rosmsg;

import ROS.RosTopicMessage;

/**
 * Gson object class: A class of a rosTopicMessage with its "msg" typed as std_msgs/String, which is a very common rosTopic message
 */

public class RosTopicMessageString extends RosTopicMessage {

    private MsgString msg;

    /**
     *Construct a object of the String rosMessage
     *
     * @param op
     * @param topic
     * @param type
     */



    private RosTopicMessageString(String op, String topic, String type){
        super(op, topic, type);
        }

    public MsgString getMsg() {
        return msg;
    }


    /**
     * Gson object class: A class of the "msg" field of a rosTopicMessage -- std_msgs/String
     */

    public class MsgString {
        private String data;
        public MsgString(String data){this.data = data;}

        public String getData() {
            return data;
        }
    }


    /**
     * This method build a std_msgs/String RosMessage object
     * @param op publish
     * @param topic The string of the topic name
     * @param type std_msgs/String
     * @param msg The String Message to be send to the topic
     * @return An object of class RosTopicMessageString(std_msgs/String)
     */
    public static RosTopicMessageString buildRosTopicMessageString(String op, String topic, String type, String msg){
        RosTopicMessageString temp = new RosTopicMessageString(op, topic, type);
        temp.msg = temp.new MsgString(msg);
        return temp;
    }

}


