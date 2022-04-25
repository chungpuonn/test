package transforma.qb2_user_app.rosmsg;

import android.util.Log;
import android.widget.Toast;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;

import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.List;

import ROS.*;
import transforma.qb2_user_app.ConfigurationParameters;
import transforma.qb2_user_app.QuicabotAbout;
import transforma.qb2_user_app.R;
import transforma.qb2_user_app.Utils;
import transforma.qb2_user_app.quicabotInspectionLog.InspectionLog;


/**
 * <li>This is a singleton class of inspection result that only creates itself when the inspection starts</li>
 * <li>It's created to store relevant numbers relating to inspection result.</li>
 * <li>Check whether the inspectionMode result attribute is null or not to determine whether there is an ongoing inspection.</li>
 * <li>Making inspectionResult null pointer is necessary when an inspection finishes.</li>
 * <li>Adding & Removing RosBridgeListener is necessary to prevent error occuring</li>
 *
 */

public class InspectionResult implements RosBridgeListener {
    /**
     * <li>This number is used to find out exactly how many points the point cloud information from the raw message string contains</li>
     * <li>This 25.8 is a rough number form about tens of sample data string.</li>
     * <li>This number cant be larger as it would lead to insufficient buffer size allocated to points</li>
     * <li>Smaller number would lead to unused buffer as points are stored in buffer for optimising running</li>
     */
    private static final float DIVIDING_FACTOR = ConfigurationParameters.DIVIDING_FACTOR;
    private Date startDateTime;
    public ArrayList<String> jobCreated;
    public boolean isReportedGenerated;
    public Integer crackCount;
    public Integer lippageCount;
    public Integer unevenCount;
    public Integer totalDefectsCount;
    public Integer misalignCount;
    private static volatile InspectionResult inspectionResult;
    private boolean isRobotWorking = false;

    public FloatBuffer buffer = null;

    /**
     * Check for isthere is onGoing inspection
     * @return
     */
    public static boolean onGoingInspection(){
        return inspectionResult != null;
    }

    /**
     * End an inspection recording by null reference the attribute
     * and removing the listener
     */

    public void endInspection(){

        inspectionResult = null;
        RosBridgeConnection.getRosBridgeConnection().removeListeners(this);
    }

    /**
     * TODO: finish this function
     * This method read log file and store current inspection into log
     * @param endDateTime
     */
    public List<InspectionLog> logInspection(Date endDateTime, List<InspectionLog> oldlog) {

        InspectionLog log = new InspectionLog(startDateTime.toString(), endDateTime.toString(),
                jobCreated, totalDefectsCount, crackCount, unevenCount, misalignCount, isReportedGenerated);
        oldlog.add(log);
        return oldlog;

    }

    /**
     * Start a new inspection recording or get the current inspection recording
     * @return
     */

    public static InspectionResult getInspectionResult() {
        if (inspectionResult == null){
            inspectionResult = new InspectionResult();
        }
        return inspectionResult;
    }



    /**
     * Private constructor for singleton class
     */

    private InspectionResult(){
        crackCount = totalDefectsCount = unevenCount = misalignCount = lippageCount = 0;
        this.startDateTime = Calendar.getInstance().getTime();
        this.jobCreated = new ArrayList<>();
        RosBridgeConnection.getRosBridgeConnection().addListeners(this, "inspectionMode result");
    }

    @Override
    public void onMessage(boolean bool) {
        return;
    }

    /**
     * This method is a large onMessage method basically to update and display inspection information onto the screen
     * Case by case review is necessary
     *
     * Note: For case pointCloud, parsing is not done by loading entire string into memory to form large class objects as it generate tens of thousands,
     * of RosTopicMessage_MsgGeometry_Point object which would crash the java heap size and may lead to GC of other critical class objects such as RosBridgeConnection
     * Paring is done using jsonReader which implements streamReader. This helps to maintain a low memory usage. Note that for other large data blocks, such method should also be used
     *
     * @param message Raw message string
     * @param rosMsgRaw rawMessage object with op field
     */

    @Override
    public void onMessage(String message, RosMsgRaw rosMsgRaw) {

        if (rosMsgRaw.getOp().equals("publish")) {
            RosTopicMessage rosTopicMessage = new Gson().fromJson(message, RosTopicMessage.class);
            switch (rosTopicMessage.getTopic()) {
                case TOPIC_CRACK:
                    //TODO: If storing of location required, needs to additional method
                    RosTopicMessageCrackResult rosTopicMessageCrackResult = new Gson().fromJson(message, RosTopicMessageCrackResult.class);
                    if (rosTopicMessageCrackResult.getMsg().getCrack_result() == 1){
                        crackCount++;
                        totalDefectsCount++;

                    }
                    break;
                case TOPIC_UNEVENMISALIGN:
                    RosTopicMessageMisAlignLog rosMessageMisAlignLog = new Gson().fromJson(message, RosTopicMessageMisAlignLog.class);
                    if (rosMessageMisAlignLog.getMsg().getMisalignment_result() == 1) {
                        misalignCount++;
                        totalDefectsCount++;
                    }
                    else if (rosMessageMisAlignLog.getMsg().getUneven_result() == 1){
                        unevenCount++;
                        totalDefectsCount++;
                    }
                    break;
                case TOPIC_LIPPAGE:
                    RosTopicMessageLippageLog rosMessageLippageLog = new Gson().fromJson(message, RosTopicMessageLippageLog.class);
                    if (rosMessageLippageLog.getMsg().getLippage_result() == 1){
                        lippageCount++;
                        totalDefectsCount++;
                    }
                    break;
                case TOPIC_POINTCLOUD:
                    //TODO: Think of a better way to initialise array
                    //FORMAT: {"topic": "downsampled_pointcloud_gui", "msg": {"header": {"stamp": {"secs": 0, "nsecs": 0}, "frame_id": "", "seq": 7}, "points": [{"y": 0.0, "x": 0.0, "z": 0.0}, ...], "op": "publish"}
                    //Log.i("String Length", Integer.toString(message.length()));
                    float[] points = new float[(int)(message.length() / DIVIDING_FACTOR)];   // Creates a float[] for point storage, DIVIDING_FACTOR comes from estimated calculation
                    JsonReader reader = new JsonReader(new StringReader(message));
                    try {
                        reader.beginObject();
                        while (reader.hasNext()){
                            String name = reader.nextName();
                            if (name.equals("msg")){ // Read 'msg' field
                                reader.beginObject();
                                while (reader.hasNext()){
                                    if (reader.nextName().equals("points")) // Read 'points' field
                                        readFloatArray(reader, points);
                                    else reader.skipValue();
                                }
                                reader.endObject();
                            } else reader.skipValue();
                        }
                        reader.endObject();
                        buffer = null; // Recycle previous data
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    buffer = ByteBuffer.allocateDirect(points.length * 4 ).order(ByteOrder.nativeOrder()).asFloatBuffer(); // Allocate buffer
                    buffer.put(points).position(0);
                    points = null; // Force gc
                    break;
                case TOPIC_ROBOT_STATE:
                    final RosTopicMessageString robotState = new Gson().fromJson(message, RosTopicMessageString.class);
                    switch (robotState.getMsg().getData()){
                        case "initialising":
                            break;
                        case "idle":
                            inspectionResult.setRobotWorking(false);
                            break;
                        case "running":
                            inspectionResult.setRobotWorking(true);
                            break;
                        case "error":
                    }
                    break;
            }
        } else if (rosMsgRaw.getOp().equals("service_response")) {
//            RosServiceMessage rosSrvMessage= new Gson().fromJson(message, RosServiceMessage.class);
//            switch (rosSrvMessage.getService()){
//                case "":
//            }
        }

    }

    /**
     * THis method reads the points field of the RosMessage for pointCloud data and converts it into a float[]
     * @param reader  Json reader to be passed in where positioned at array location.
     * @param point Float array to store data
     * @return float array with data
     * @throws IOException the reader may not positioned at array beginning location and thus may throw IOException
     * @throws ArrayIndexOutOfBoundsException The array to store points may not big enough and throws ArrayOutOfBoundsException
     */


    public static float[] readFloatArray(JsonReader reader, float[] point) throws IOException, ArrayIndexOutOfBoundsException{
        int y = 0;
        reader.beginArray();
        while (reader.hasNext()){
            reader.beginObject();
            while (reader.hasNext()){
                if (reader.nextName().equals("y")){
                    point[3 * y + 1] = -(float)reader.nextDouble();
                }
                if (reader.nextName().equals("x")){
                    point[3 * y] = (float)reader.nextDouble();
                }
                if (reader.nextName().equals("z")){
                    point[3 * y + 2] = (float)reader.nextDouble();
                }

            }
            reader.endObject();
            y++;
        }
        //Log.i("Total points", Integer.toString(y));
        reader.endArray();
        return point;
    }


    public boolean isRobotWorking() {
        return isRobotWorking;
    }

    public void setRobotWorking(boolean robotWorking) {
        isRobotWorking = robotWorking;
    }
}
