package transforma.qb2_user_app.rosmsg;

import ROS.RosTopicMessage;
import ROS.RosTopicMessage_MsgGeometry_Twist;
import ROS.RosTopicMessage_MsgGeometry_Vector3;

public class RosTopicMessageCmdVel extends RosTopicMessage {

    private RosTopicMessage_MsgGeometry_Twist msg;

    private RosTopicMessageCmdVel(String op, String topic, String type, RosTopicMessage_MsgGeometry_Twist twist) {
        super(op, topic, type);
        msg = twist;
    }


    public RosTopicMessage_MsgGeometry_Twist getMsg() {
        return msg;
    }

    public void setMsg(RosTopicMessage_MsgGeometry_Twist msg) {
        this.msg = msg;
    }


    public static RosTopicMessageCmdVel buildRosTopicMessageCmdVel(String op, String topic, String type, double linear_velocity_x, double angular_velocity_z){
        RosTopicMessage_MsgGeometry_Vector3 linear = new RosTopicMessage_MsgGeometry_Vector3(linear_velocity_x, 0.0, 0.0);
        RosTopicMessage_MsgGeometry_Vector3 angular = new RosTopicMessage_MsgGeometry_Vector3(0.0,0.0,angular_velocity_z);
        RosTopicMessage_MsgGeometry_Twist twist = new RosTopicMessage_MsgGeometry_Twist(linear, angular);
        return new RosTopicMessageCmdVel(op, topic, type, twist);


    }


}
