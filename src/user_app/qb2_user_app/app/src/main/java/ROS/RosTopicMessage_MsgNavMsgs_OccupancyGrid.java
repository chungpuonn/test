package ROS;


/**
 * Gson object class: A class of the "msg" field of a rosTopicMessage, "nav_msgs/OccupancyGrid" is a rosMessage type,
 * Refer to <a href="http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html">Ros Documentation</a>.
 */

public class RosTopicMessage_MsgNavMsgs_OccupancyGrid {


    private RosTopicMessage_MsgStd_Header header;

    /**
     * The map data, in row-major order, starting with (0,0).  Occupancy
     * probabilities are in the range [0,100].  Unknown is -1.
     */
    private byte[] data;
    private RosTopicMessage_MsgNavMsgs_MapMetaData  info;

    public RosTopicMessage_MsgStd_Header getHeader() {
        return header;
    }

    public void setHeader(RosTopicMessage_MsgStd_Header header) {
        this.header = header;
    }

    public byte[] getData() {
        return data;
    }

    public void setData(byte[] data) {
        this.data = data;
    }

    public RosTopicMessage_MsgNavMsgs_MapMetaData getInfo() {
        return info;
    }

    public void setInfo(RosTopicMessage_MsgNavMsgs_MapMetaData info) {
        this.info = info;
    }
}
