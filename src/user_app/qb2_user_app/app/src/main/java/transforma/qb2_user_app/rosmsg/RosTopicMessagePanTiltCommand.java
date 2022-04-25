package transforma.qb2_user_app.rosmsg;

import ROS.RosTopicMessage;
import ROS.RosTopicMessage_MsgTrajectory_JointTrajectoryPoints;
import ROS.RosTopicMessage_MsgTrajectory_JointTrajectory;

/**
 * Gson object class: A class of a PicaBot specified rosMessage with its "msg" typed as trajectory points, which is a special type of rsoMessage.
 * This class is used to contain information for setting pan-tilt system position information
 */

public class RosTopicMessagePanTiltCommand extends RosTopicMessage {

    private RosTopicMessage_MsgTrajectory_JointTrajectory msg;


    private RosTopicMessagePanTiltCommand(String op, String topic, String type, RosTopicMessage_MsgTrajectory_JointTrajectory msg) {
        super(op, topic, type);
        this.msg = msg;
    }

    public RosTopicMessage_MsgTrajectory_JointTrajectory getMsg() {
        return msg;
    }



    /**
     * This method build a std_msgs/String RosMessage object
     * @param op publish
     * @param topic The string of the topic name
     * @param type std_msgs/String
     * @param jointNames ["pan" , "tilt"]
     * @param positions [{&pan}, {&tilt}]
     * @return An object of class RosTopicMessageString(std_msgs/String)
     */
    public static RosTopicMessagePanTiltCommand buildRosTopicMessageTrajectoryMessageforPanTilt(String op, String topic, String type, String[] jointNames, float[] positions){
        RosTopicMessage_MsgTrajectory_JointTrajectoryPoints tempMsgPoints = new RosTopicMessage_MsgTrajectory_JointTrajectoryPoints(positions);
        RosTopicMessage_MsgTrajectory_JointTrajectory tempMsgTrajectory = new RosTopicMessage_MsgTrajectory_JointTrajectory(jointNames,tempMsgPoints);
        return new RosTopicMessagePanTiltCommand(op, topic, type, tempMsgTrajectory);
    }
}
