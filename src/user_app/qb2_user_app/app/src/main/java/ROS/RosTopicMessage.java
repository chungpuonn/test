package ROS;

/**
 * <p>
 *     Gson object class: A general class of RosTopicMessage for all other type of Ros Topic message Json serialisation and de-serialisation
 * </p>
 *
 * Naming convention:
 * For subclasses, the entire string received through the socket, use RosMessage12345 as class name
 * For the message itself, the json string inside msg field, use Msg12345 corresponding to class name above
 *
 */

public class RosTopicMessage extends RosMsgRaw{

    private String topic;
    private String type;

    public RosTopicMessage(String op, String topic, String type){
        super(op);
        this.topic = topic; this.type = type;
    }

    /**
     * Get topic name
     * @return
     */


    public String getTopic() {
        return topic;
    }

    /**
     * Get type name
     * @return
     */

    public String getType() {
        return type;
    }
}
