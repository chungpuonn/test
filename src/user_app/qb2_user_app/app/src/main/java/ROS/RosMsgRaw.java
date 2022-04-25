package ROS;

/**
 * Gson object class: Raw RosMessage it can be a rosTopic message or service message as it only has the op field
 */

public class RosMsgRaw {

    private String op;

    /**
     * Construct a Raw RosMessage
     * @param op op of the message
     */

    public RosMsgRaw(String op){
        this.op = op;
    }

    /**
     * Get the op of the message
     * @return
     */

    public String getOp() {
        return op;
    }

}
