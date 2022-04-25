package transforma.qb2_user_app.rosmsg;

import ROS.*;

/**
 * Gson object class: A class of a rosSrvMessageAddress with its "args"(for service call) typed as the Address string, and its "values"(roe service response) typed as a int32,
 * 0 for not received and 1 for received
 *
 */

public class RosServiceMessageAddress extends RosServiceMessage {

    RosSrvMessage_MsgAddress args;
    RosSrvMessage_MsgAddressResponse values;

    public RosServiceMessageAddress(String op, String service) {
        super(op, service);
    }

    public void setArgs(RosSrvMessage_MsgAddress args) {
        this.args = args;
    }

    public class RosSrvMessage_MsgAddress {
        public String address;
        public RosSrvMessage_MsgAddress(String address) {
            this.address = address;
        }
    }
    public class RosSrvMessage_MsgAddressResponse {
        public int isReceived;
    }


    /**
     * This method builds a RosServiceMessage specifically for the address service call
     * @param op call_service
     * @param service The string of the service name
     * @param address The string of the inspection address
     * @return An object of the class RosServiceMessageAddress
     */
    public static RosServiceMessageAddress buildRosSrvMessageAddress(String op, String service, String address){
        RosServiceMessageAddress temp = new RosServiceMessageAddress(op, service);
        temp.setArgs(temp.new RosSrvMessage_MsgAddress(address));
        return temp;
    }
}
