package ROS;

/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "nav_msgs/MapMetaData" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html">Ros Documentation</a>.
 */

public class RosTopicMessage_MsgNavMsgs_MapMetaData {


    private MsgTime time;
    //Note that original height and width are uint32, which means only long is compatible under full use. We use int for simplified version
    private int width, height; // cell

    private float resolution; // m/cell
    /**
     * The origin of the map [m, m, rad].  This is the real-world pose of the
     * cell (0,0) in the map.
     */
    private RosTopicMessage_MsgGeometry_Pose origin;


    private class MsgTime{
        public int secs;
        public int nsecs;
    }


    public MsgTime getTime() {
        return time;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public float getResolution() {
        return resolution;
    }

    public RosTopicMessage_MsgGeometry_Pose getOrigin() {
        return origin;
    }
}
