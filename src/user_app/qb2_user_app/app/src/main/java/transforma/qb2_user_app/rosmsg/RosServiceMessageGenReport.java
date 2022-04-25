package transforma.qb2_user_app.rosmsg;

import ROS.RosServiceMessage;

/**
 * Gson object class: A class of a rosSrvMessage with its "args"(for service call) typed as the command string, and its "values"(ros service response) typed as a int32,
 * 0 for not generated and 1 for generated
 *
 */

public class RosServiceMessageGenReport extends RosServiceMessage {

    RosSrvMessage_MsgGenReport args;
    RosSrvMessage_MsgGenReportResponse values;

    public RosServiceMessageGenReport(String op, String service) { super(op, service); }

    public void setArgs(RosSrvMessage_MsgGenReport args) { this.args = args; }

    public class RosSrvMessage_MsgGenReport {
        String cmd;

        public RosSrvMessage_MsgGenReport(String cmd) { this.cmd = cmd; }
    }
    public class RosSrvMessage_MsgGenReportResponse {
        public int getIsGenerated() {
            return isGenerated;
        }

        int isGenerated;

    }

    public RosSrvMessage_MsgGenReportResponse getValues() {
        return values;
    }

    /**
     * This method builds a RosServiceMessage specifically for the generating resport service call
     * @param op call_service
     * @param service The string of the service name
     * @param cmd The instruction string to generate report
     * @return An object of the class RosServiceMessageGenReport
     */

    public static RosServiceMessageGenReport buildRosSrvMessageGenReport(String op, String service, String cmd){
        RosServiceMessageGenReport temp = new RosServiceMessageGenReport(op, service);
        temp.setArgs(temp.new RosSrvMessage_MsgGenReport(cmd));
        return temp;
    }
}
