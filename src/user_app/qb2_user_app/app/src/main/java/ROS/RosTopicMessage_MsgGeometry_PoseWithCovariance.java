package ROS;

public class RosTopicMessage_MsgGeometry_PoseWithCovariance {


    RosTopicMessage_MsgGeometry_Pose pose;

    //Note that original data is a float64 and we simplified the data to float
    //float[] covariance;


    public RosTopicMessage_MsgGeometry_Pose getPose() {
        return pose;
    }

    public void setPose(RosTopicMessage_MsgGeometry_Pose pose) {
        this.pose = pose;
    }
}
