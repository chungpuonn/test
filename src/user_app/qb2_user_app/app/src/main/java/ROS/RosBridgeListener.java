package ROS;

/**
 * This is a listener interface that all Android Cctivity classes should implement as long as it want to receive message with Rosbridge.
 */

public interface RosBridgeListener {


    String CANCEL = "Cancel";
    /**
     * Shared Preference Name
     */
    String PREFERENCE_NAME = "qb2_user_app_preference";
    /**
     * WebSocket prefix, change according to robot model
     */
    String SOCKETSTART = "ws://192.";
    /**
     * Http prefix
     */
    String HTTPSTART = "http://192.";
    /**
     * RosBridge Suffix
     */
    String STRINGPORT = ":9090";
    /**
     * WebVideoServer suffix
     */
    String IMAGEPORT = ":8080";

    String PUBLISH = "publish";
    String CALL_SERVICE = "call_service";
    String SERVICE_RESPONSE = "service_response";
    /**
     * Working mode for quicabot, may edit according to what we have
     */
    String[] WORKING_MODE = {"wall auto", "wall col", "single", "floor"};
//    static final String[] ROBOT_STATE = {"initialising", "idle", "running", "error"};

    /**
     * All other Topic and Type Strings
     */
    String SERVICE_ADDR = "inspection_address";
    String SERVICE_REPORT_GEN = "report_generation";
    String TOPIC_OPMODE = "operation_mode";
    String TYPE_STRING = "std_msgs/String";
    String TOPIC_PAN_TILT_COMMAND = "pan_tilt/command";
    String TOPIC_PAN_TILT_STATE = "pan_tilt/state";
    String TYPE_TRAJECTORYMSG = "trajectory_msgs/JointTrajectory";
    String TYPE_SENSOR_MSGS_JOINT_STATE = "sensor_msgs/JointState";
    String TOPIC_CRACK = "crack_detection/crack_result";
    String TYPE_CRACK = "quicabot_msgs/CrackResult";
    String TOPIC_UNEVENMISALIGN = "uneven_misalign_lippage_detection/uneven_misalign_log";
    String TYPE_UNEVENMISALIGN = "quicabot_msgs/UnevenMisalignResult";
    String TOPIC_LIPPAGE = "uneven_misalign_lippage_detection/lippage_log";
    String TYPE_LIPPAGE = "quicabot_msgs/LippageResult";
    String TOPIC_POINTCLOUD = "downsampled_pointcloud_gui";
    String TYPE_POINTCLOUD = "quicabot_msgs/PointArray";
    String TOPIC_COMMAND = "interrupt_command";
    String TOPIC_ROBOT_STATE = "robot_status/command";
    String TOPIC_CMD_VEL = "cmd_vel";
    String TYPE_GEOMETRY_VECTOR3 = "geometry_msgs/Twist";
    String TOPIC_MAP = "map";
    String TYPE_NAVMSGS_OCCUPANCYGRID = "nav_msgs/OccupancyGrid";
    String TOPIC_POSEUPDATE = "poseupdate_thottled";
    String TYPE_GEOMETRY_POSEWITHCOVARIANCESTAMPED = "geometry_msgs/PoseWithCovarianceStamped";
    /**
     * Caack webVideoServer suffix
     */
    static final String URLIMAGECRACK = "/stream?topic=/crack_detection/result_image";
    /**
     * MisAlign webVideoServer suffix
     */
    static final String URLIMAGEUNEVEN = "/stream?topic=/uneven_misalign_lippage_detection/uneven_misalign_image";
    /**
     * Lippage webVideoServer suffix
     */
    static final String URLLIPPAGE = "/stream?topic=/uneven_misalign_lippage_detection/lippage_image";

    public static final String ROS_BRIDGE_ERROR = "RosBridge Error";

    /**
     * This method is called when RosBridge itself connected or disconnected regardless what caused it, it is to warn the respective listener the connection status
     * @param bool true when connection establishes, false when connection attempt fails or connection breaks
     */


    void onMessage(boolean bool);

    /**
     * This method is called when RosBridge receives a message,
     * @param message the raw string of the entire message
     * @param msgRaw The processed message string containing op infomation and it helps to tell whether the message is a service or topic
     */


    void onMessage(String message, RosMsgRaw msgRaw);
}
