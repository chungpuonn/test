package ROS;

/**
 * Gson object class: RosServiceMessage is a service message can be either a call or a response
 */

public class RosServiceMessage extends RosMsgRaw {

    private String service;
    private boolean result;

    /**
     * Construct the Service Messsage
     * @param op op
     * @param service service name
     */

    public RosServiceMessage(String op, String service) {
        super(op);
        this.service = service;
    }

    /**
     * Get service name
     * @return
     */


    public String getService() {
        return service;
    }

    /**
     * Get service result
     * @return "true" means the the response message op is "service_response" and "false" means the reponse message op is still "call_service"
     */


    public boolean getResult() {
        return result;
    }
}
