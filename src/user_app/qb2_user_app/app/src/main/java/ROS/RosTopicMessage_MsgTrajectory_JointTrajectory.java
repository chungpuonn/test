package ROS;

/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "Trajectory_JointTrajectory" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html">Ros Documentation</a>.
 */
public class RosTopicMessage_MsgTrajectory_JointTrajectory {

    RosTopicMessage_MsgStd_Header header;

    public RosTopicMessage_MsgStd_Header getHeader() {
        return header;
    }

    String[] joint_names;

    RosTopicMessage_MsgTrajectory_JointTrajectoryPoints[] points;

    public RosTopicMessage_MsgTrajectory_JointTrajectory(String[] joint_names, RosTopicMessage_MsgTrajectory_JointTrajectoryPoints points) {
        this.joint_names = joint_names;
        this.points = new RosTopicMessage_MsgTrajectory_JointTrajectoryPoints[]{points};
    }
}
