package transforma.qb2_user_app.quicabotMap;

import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Switch;
import android.widget.Toast;

import com.google.gson.Gson;
import com.xw.repo.BubbleSeekBar;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;

import ROS.RosMsgRaw;
import ROS.RosTopicMessage;
import ROS.RosTopicMessage_MsgGeometry_Point;
import ROS.RosTopicMessage_MsgGeometry_PoseWithConvarianceStamped;
import ROS.RosTopicMessage_MsgGeometry_Quaternion;
import ROS.RosTopicMessage_MsgNavMsgs_MapMetaData;
import ROS.RosTopicMessage_MsgNavMsgs_OccupancyGrid;
import io.github.controlwear.virtual.joystick.android.JoystickView;
import transforma.qb2_user_app.ConfigurationParameters;
import transforma.qb2_user_app.QuicabotAppCompatActivity;
import transforma.qb2_user_app.R;
import transforma.qb2_user_app.Utils;
import transforma.qb2_user_app.rosmsg.RosTopicMessageCmdVel;
import transforma.qb2_user_app.rosmsg.RosTopicMessageMap;
import transforma.qb2_user_app.rosmsg.RosTopicMessagePoseUpdate;

public class QuicabotMapPage extends QuicabotAppCompatActivity {

    private static final double VEL_MAXLINEAR = ConfigurationParameters.VEL_MAXLINEAR; //The maximum linear velocity of the robot moving system
    private static final double VEL_MAXANGULAR = ConfigurationParameters.VEL_MAXANGULAR; //The maximun rotation velocity of the robot moving system

    private MapView mapView;
    private MapRenderer mapRenderer;


    static FloatBuffer mapBuffer; // Storing the point location information of the detected map with color opacity information
    static FloatBuffer robotBuffer; // Storing the point locations that forms an arrow representing the current robot location and direction
    float[] pointData;
    static int pointCount = 0;

    static boolean lock = false;
    static boolean robotDataLock = true;
    static boolean firstLaunch = true;

    private Switch baseMoveSwitch;  //Switch that whether enable the progress bar

    private JoystickView myJoyStick;







    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);



        final DisplayMetrics displayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displayMetrics); //Getting screen resolution information

        setContentView(R.layout.activity_quicabot_map_page);
        mapView = findViewById(R.id.mapView);
        mapRenderer = new MapRenderer(this, findViewById(R.id.mapLegend));
        mapView.init(mapRenderer, displayMetrics.density);

        myJoyStick = findViewById(R.id.joystickView);
        baseMoveSwitch = findViewById(R.id.baseControlSwitch);
        //This set the listener of the joystick.
        myJoyStick.setOnMoveListener(this::moveTheRobot);




    }

    @Override
    protected void onStart() {
        super.onStart();
        getMapSaved(); //Getting saved map from the internal storage
    }

    @Override
    protected void onResume()
    {
        // The activity must call the GL surface view's onResume() on activity onResume().
        super.onResume();
        mapView.onResume();
    }

    @Override
    protected void onPause()
    {
        // The activity must call the GL surface view's onPause() on activity onPause().
        super.onPause();
        mapView.onPause();
    }


    @Override
    public void onBackPressed() {
        if (pointData != null)
            saveMapData(new MapData(pointData)); //Save map to internal storage when exiting the page
        try {
            rosBridgeConnection.unSubscribe(TOPIC_MAP, TYPE_NAVMSGS_OCCUPANCYGRID);
            rosBridgeConnection.unSubscribe(TOPIC_POSEUPDATE, TYPE_GEOMETRY_POSEWITHCOVARIANCESTAMPED);

        } catch (Exception e) {
            Log.i(ROS_BRIDGE_ERROR, e.getMessage());
            Toast.makeText(this, ROS_BRIDGE_ERROR, Toast.LENGTH_SHORT).show();
        }
        super.onBackPressed();
    }

    /**
     * This method is called when RosBridge receives a message,
     *
     * @param message the raw string of the entire message
     * @param rosMsgRaw  The processed message string containing op infomation and it helps to tell whether the message is a service or topic
     */
    @Override
    public void onMessage(String message, RosMsgRaw rosMsgRaw) {

        if (rosMsgRaw.getOp().equals("publish")) {
            RosTopicMessage rosTopicMessage = new Gson().fromJson(message, RosTopicMessage.class);
            switch (rosTopicMessage.getTopic()) {
                case TOPIC_MAP:

                    RosTopicMessageMap mapTemp = new Gson().fromJson(message, RosTopicMessageMap.class);
                    RosTopicMessage_MsgNavMsgs_OccupancyGrid mapGrid = mapTemp.getMsg();
                    float robotX = mapGrid.getInfo().getOrigin().getPosition().x;
                    float robotY = mapGrid.getInfo().getOrigin().getPosition().y;

                    Runnable runnable = () -> {
                        byte[] data = mapGrid.getData();
                        while (lock) {
                            try {
                                Thread.sleep(20);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                        lock = true;
                        generateMapFloatBuffer(data);
                        lock = false;
                        firstLaunch = false;
                    };
                    runnable.run();
                    break;
                case TOPIC_POSEUPDATE:
                    RosTopicMessagePoseUpdate robotPose = new Gson().fromJson(message, RosTopicMessagePoseUpdate.class);
                    RosTopicMessage_MsgGeometry_Point position = robotPose.getMsg().getPose().getPose().getPosition();
                    RosTopicMessage_MsgGeometry_Quaternion direction = robotPose.getMsg().getPose().getPose().getOrientation();
                    generateRobotPositionBuffer(position, direction);
                    break;


            }
        } else if (rosMsgRaw.getOp().equals("service_response")) {
//            RosServiceMessage rosSrvMessage= new Gson().fromJson(message, RosServiceMessage.class);
//            switch (rosSrvMessage.getService()){
//                case "":
//            }
        }

    }

    private void generateMapFloatBuffer(byte[] data) {
        //Read relevant rosTopic information and (TurtleBot rosNode configuration)
        ArrayList<Float> tempPointData = new ArrayList<>();
        for (int i = 0; i < data.length; i++) {
            if (data[i] >= 0){ // Pixel with in detection range would be greater than 0, ranging from 0-100 representing probability of blocking
                float x = i % MapView.DEFINITION_X * MapView.RESOLUTION;
                float y = i / MapView.DEFINITION_X * MapView.RESOLUTION;
                float z = 0f;
                float red, green, blue;
                float alpha = 1f;
                if (data[i] > 85){ //Color for fully occupied blk when chance is greater than 85/100
                    red= 0.98f;
                    green = 0.95f;
                    blue = 0.8f;
                } else {  //Color for
                    red= 0.949019f;
                    green = 0.78039f;
                    blue = 0.43529f;
                }
                tempPointData.addAll(Arrays.asList(x,y,z,red,green,blue,alpha));
            }


        }
        pointCount = tempPointData.size();
        pointData = new float[pointCount];
        int i = 0;

        //Converting an ArrayList to array
        for (Float f : tempPointData) {
            pointData[i++] = f;

        }

        //MAke floatbuffer
        mapBuffer = ByteBuffer.allocateDirect(pointData.length*4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mapBuffer.put(pointData).position(0);




    }

    private void generateRobotPositionBuffer(RosTopicMessage_MsgGeometry_Point position, RosTopicMessage_MsgGeometry_Quaternion direction){


        float markZoom = 0.3f; //Take change of how sharp the arrow is
        float markZoomBot = 0.15f; //Take charge of how large the arrow bottom is
        float x = position.x + MapView.INITIAL_X;
        float y = position.y + MapView.INITIAL_Y;
        float z = 0;

        double angle = 2 * Math.acos(direction.w);
        double zRotation = ((1- direction.w*direction.w) < 0.001 ? (direction.z) : (direction.z /  Math.sqrt(1-direction.w*direction.w)) ) * angle;

        //Try read ur self...
        Log.i("Rotation", String.valueOf(angle));
        float dy0 = (float)Math.sin(zRotation);
        float dx0 = (float) Math.cos(zRotation);
        float dy1 = (float)Math.sin(zRotation + 2.09439);
        float dx1 = (float) Math.cos(zRotation + 2.09439);
        float dy2 = (float)Math.sin(zRotation + 4.1888);
        float dx2 = (float) Math.cos(zRotation + 4.1888);

        float[] robotData = {x, y, z,        x + markZoom * dx0, y + markZoom * dy0, z,       x + markZoomBot * dx1, y + markZoomBot * dy1, z,
                             x, y, z,        x + markZoom * dx0, y + markZoom * dy0, z,       x + markZoomBot * dx2, y + markZoomBot * dy2, z,};

        robotDataLock = true;


        robotBuffer = ByteBuffer.allocateDirect(robotData.length*4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        robotBuffer.put(robotData).position(0);

        robotDataLock = false;

    }

    //Retrieve saved map into the programme
    private void getMapSaved() {
        lock = true;
        MapData data;
        String logDataString = Utils.readFile(this, ConfigurationParameters.mapFileName);
        if (logDataString != null){


            data = new Gson().fromJson(logDataString, MapData.class);
            pointData = data.mapDataFloat;
            pointCount = pointData.length;
            mapBuffer = ByteBuffer.allocateDirect(pointData.length*4).order(ByteOrder.nativeOrder()).asFloatBuffer();
            mapBuffer.put(pointData).position(0);
            firstLaunch = false;
        }
        lock = false;


    }

    //Save the map information to internal storage
    private void saveMapData(MapData data) {
        String inspectionLogString = new Gson().toJson(data);
        Utils.writeFIle(this, inspectionLogString, ConfigurationParameters.mapFileName);
    }

    public void SwitchingControl(View view) {
        myJoyStick.setEnabled(baseMoveSwitch.isEnabled());
    }

    public void generateMap(View view) {
    }


    /**
     * This method is private method to be called for joystick listener
     * @param angle the displacement angle on the virtual joystick
     * @param strength the displacement extent on the virtual joystick
     */

    private void moveTheRobot(int angle, int strength) {
        double linear = Math.sin(Math.toRadians(angle)) * strength / 100 * VEL_MAXLINEAR;
        double angular = - Math.cos(Math.toRadians(angle)) * strength /100 * VEL_MAXANGULAR;

        RosTopicMessageCmdVel temp= RosTopicMessageCmdVel.buildRosTopicMessageCmdVel(PUBLISH, TOPIC_CMD_VEL, TYPE_GEOMETRY_VECTOR3, linear, angular);
        rosBridgeConnection.publish(temp);
    }

}
