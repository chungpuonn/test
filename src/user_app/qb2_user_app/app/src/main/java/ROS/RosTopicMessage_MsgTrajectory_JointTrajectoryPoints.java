package ROS;


/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "Trajectory_JointTrajectoryPoints" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html">Ros Documentation</a>.
 *
 * Each trajectory point specifies either positions[, velocities[, accelerations]]
 * or positions[, effort] for the trajectory to be executed.
 * All specified values are in the same order as the joint names in JointTrajectory.msg
 */

public class RosTopicMessage_MsgTrajectory_JointTrajectoryPoints {
    //Note that a double is required in actual use. We use a shortened float for optimised memory
    float[] positions, velocities, accelerations, effort;

    public RosTopicMessage_MsgTrajectory_JointTrajectoryPoints(float[] positions) {
        this.positions = positions;
    }

}
