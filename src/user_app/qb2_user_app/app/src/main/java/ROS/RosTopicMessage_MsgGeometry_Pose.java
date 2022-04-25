package ROS;

public class RosTopicMessage_MsgGeometry_Pose {

    private RosTopicMessage_MsgGeometry_Point position;
    private RosTopicMessage_MsgGeometry_Quaternion orientation;


    public RosTopicMessage_MsgGeometry_Point getPosition() {
        return position;
    }

    public void setPosition(RosTopicMessage_MsgGeometry_Point position) {
        this.position = position;
    }

    public RosTopicMessage_MsgGeometry_Quaternion getOrientation() {
        return orientation;
    }

    public void setOrientation(RosTopicMessage_MsgGeometry_Quaternion orientation) {
        this.orientation = orientation;
    }
}
